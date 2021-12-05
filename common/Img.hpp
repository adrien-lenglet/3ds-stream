#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <memory>
#include <variant>
#include <string>
#include <sstream>
#include <mutex>
#include <thread>
#include <optional>
#include <tuple>
#include <condition_variable>

static size_t align_up(size_t v, size_t al)
{
	size_t mod = v % al;
	if (mod == 0)
		return v;
	else
		return v - mod + al;
}

template <typename T>
static T min(T a, T b)
{
	if (a < b)
		return a;
	else
		return b;
}

template <typename T>
static T max(T a, T b)
{
	if (a > b)
		return a;
	else
		return b;
}

struct px {
	uint8_t data[3];

	px(const uint8_t *data)
	{
		for (size_t i = 0; i < 3; i++)
			this->data[i] = data[i];
	}

	size_t dst(const px &other) const
	{
		size_t res = 0;
		for (size_t i = 0; i < 3; i++) {
			int32_t d = static_cast<int32_t>(static_cast<uint32_t>(data[i])) - static_cast<int32_t>(static_cast<uint32_t>(other.data[i]));
			res += static_cast<uint32_t>(d * d);
		}
		return res;
	}

	uint16_t cf1r5g5b5(bool flag)
	{
		return (flag ? 1 : 0) +
			(static_cast<uint16_t>(data[0] & 0xF8) >> 2) +
			(static_cast<uint16_t>(data[1] & 0xF8) << 3) +
			(static_cast<uint16_t>(data[2] & 0xF8) << 8);
	}
};

struct Enc {
	size_t blk_size;
	size_t blk_stride;
	size_t blk_px_count;
	size_t w;
	size_t h;
	size_t blk_count;
	size_t blk_off[16];

	static constexpr size_t dw = 400;
	static constexpr size_t dh = 240;
	static constexpr size_t dblk_size = 4;
	static constexpr size_t bfstride = sizeof(px) * 2;

	static consteval Enc def(void)
	{
		Enc res;
		res.blk_size = dblk_size;
		res.blk_px_count = res.blk_size * res.blk_size;
		res.blk_stride = sizeof(px) * 2 + res.blk_px_count / 8;
		res.w = dw / res.blk_size;
		res.h = dh / res.blk_size;
		res.blk_count = res.w * res.h;
		size_t base[8] {
			-res.h - 1,
			-res.h,
			-res.h + 1,
			static_cast<size_t>(-1),
			0,
			1,
			res.h + 1,
			res.h
		};
		for (size_t i = 0; i < 8; i++) {
			size_t v = base[i] * bfstride;	// apply pixel stride
			for (size_t j = 0; j < 2; j++)
				res.blk_off[i * 2 + j] = v;
		}
		return res;
	}
};

struct HuffmanTable
{
	size_t max_bit_count;

	// decoding
	bool *is_sym;	// is bit sequence covered
	uint32_t *syms;	// bit sequence to symbol
	uint32_t max_sym;	// valid array ndx for is_sym and syms

	// encoding
	bool *is_sym_enc;	// is symbol covered
	uint32_t *syms_enc;	// symbol to bit sequence
	uint8_t *syms_enc_bit_count;	// bit sequence length
	uint32_t max_sym_enc;	// valid array ndx for is_sym_enc, syms_enc and syms_enc_bit_count

	size_t model_size;	// compressed size, defined if built using sym_occ

	struct SymProb {
		uint32_t occ;
		uint32_t sym;
	};

	// returns unsorted, cut to the right length list of all necessary symbols to assemble the table
	static std::vector<SymProb> canonSym(const std::vector<SymProb> &csym_occ, size_t threshold, size_t zsymb)
	{
		std::map<uint32_t, uint32_t> mp;
		bool has_one = false;
		for (auto &s : csym_occ)
			mp[s.sym * s.occ] = s.sym;	// weight by bit count
		std::vector<SymProb> res;
		res.reserve(threshold + 2);	// overshoot for potential extra one
		{
			auto end = mp.rend();
			size_t i = 0;
			for (auto it = mp.rbegin(); it != end; it++) {
				res.emplace_back(SymProb{it->first, it->second});
				if (it->second == 1)
					has_one = true;
				if (i++ >= threshold)
					break;
			}
		}
		res.insert(res.end(), SymProb{res[zsymb].occ - 1, 0});	// 0 symbol cannot have the most weight (least frequent when zsymb is max)
		if (!has_one)
			res.insert(res.end(), SymProb{res[0].occ + 1, 1});	// make sure we at least have a 1 symbol, make it obnioxiously likely if ever needed
		return res;
	}

	HuffmanTable(size_t max_bit_count, bool *is_sym, uint32_t *syms, uint32_t max_sym, bool *is_sym_enc,
		uint32_t *syms_enc, uint8_t *syms_enc_bit_count, uint32_t max_sym_enc, size_t model_size) :
		max_bit_count(max_bit_count),
		is_sym(is_sym),
		syms(syms),
		max_sym(max_sym),
		is_sym_enc(is_sym_enc),
		syms_enc(syms_enc),
		syms_enc_bit_count(syms_enc_bit_count),
		max_sym_enc(max_sym_enc),
		model_size(model_size)
	{
	}

	// threshold is the number of symbols encoded in the table
	// zsymb is the index after which symbol 0 will be inserted in range [0, threshold)
	HuffmanTable(const std::vector<SymProb> &csym_occ, size_t threshold, size_t zsymb)
	{
		struct Node {
			uint32_t p;
			std::variant<std::pair<Node*, Node*>, uint32_t> v;	// either node type, or symbol (leaf)

			size_t max_bit_count(void) const
			{
				if (std::holds_alternative<uint32_t>(v))
					return 0;
				else {
					auto &sub = std::get<std::pair<Node*, Node*>>(v);
					return 1 + max(sub.first->max_bit_count(), sub.second->max_bit_count());
				}
			}

			void write_table(HuffmanTable &dst, uint32_t cur_key, uint32_t cur_key_size)
			{
				if (std::holds_alternative<uint32_t>(v)) {
					auto s = std::get<uint32_t>(v);
					dst.is_sym_enc[s] = true;
					dst.syms_enc[s] = cur_key;
					dst.syms_enc_bit_count[s] = cur_key_size;

					cur_key += 1 << cur_key_size;
					//if (dst.is_sym[cur_key])
					//	std::printf("COLLISION at %u\n", cur_key);
					dst.is_sym[cur_key] = true;
					dst.syms[cur_key] = s;
				} else {
					#ifdef _3DSSTREAM_HOST	// exceptions disabled on target
					if (cur_key_size >= 32)
						throw std::runtime_error("Table bit depth exceeded 32. How have you done this?!");
					#endif
					auto &sub = std::get<std::pair<Node*, Node*>>(v);
					sub.first->write_table(dst, cur_key, cur_key_size + 1);
					sub.second->write_table(dst, cur_key + (1 << cur_key_size), cur_key_size + 1);
				}
			}
		};
		auto can = canonSym(csym_occ, threshold, zsymb);
		std::vector<std::unique_ptr<Node>> nodes;
		for (auto &c : can)
			nodes.emplace_back(new Node {c.occ, c.sym});
		for (size_t mx = nodes.size(); mx > 1; mx--) {
			size_t first;
			{
				size_t min = static_cast<size_t>(-1);
				for (size_t i = 0; i < mx; i++) {
					auto cp = nodes[i]->p;
					if (cp < min) {
						first = i;
						min = cp;
					}
				}
			}
			size_t second;
			{
				size_t min = static_cast<size_t>(-1);
				for (size_t i = 0; i < mx; i++) {
					if (i == first)
						continue;
					auto cp = nodes[i]->p;
					if (cp < min) {
						second = i;
						min = cp;
					}
				}
			}
			nodes.emplace_back(nodes[first].release());
			nodes.emplace_back(std::move(nodes[second]));
			auto &f = nodes.rbegin()[0];
			auto &s = nodes.rbegin()[1];
			nodes[first] = std::unique_ptr<Node>(new Node {f->p + s->p, std::pair<Node*, Node*>(&*f, &*s)});
			nodes.erase(nodes.begin() + second);
		}
		size_t max = 0;
		for (auto &s : can)
			if (s.sym > max)
				max = s.sym;
		max_sym_enc = max;
		is_sym_enc = new bool[max_sym_enc + 1] {};
		syms_enc = new uint32_t[max_sym_enc + 1] {};	// zero-init not to have undef in export
		syms_enc_bit_count = new uint8_t[max_sym_enc + 1] {};	// ZINUE

		max_bit_count = nodes[0]->max_bit_count();
		max_sym = (2 << (max_bit_count + 1)) - 1;	// extra redundant bit set to 1 to distinguish MSB zeroes
		is_sym = new bool[max_sym + 1] {};
		syms = new uint32_t[max_sym + 1] {};	// ZINUE

		nodes[0]->write_table(*this, 0, 0);

		model_size = 0;
		for (auto &s : csym_occ) {
			uint32_t size = 0;
			uint32_t acc = s.sym;
			while (true) {
				uint32_t sm;
				for (sm = min(acc, max_sym_enc);; sm--)
					if (is_sym_enc[sm])
						break;
				size += syms_enc_bit_count[sm];
				acc -= sm;
				if (acc == 0)
					break;
				size += syms_enc_bit_count[0];
			}

			model_size += size * s.occ;
		}
		model_size /= 8;
	}

	~HuffmanTable()
	{
		delete[] is_sym;
		delete[] syms;

		delete[] is_sym_enc;
		delete[] syms_enc;
		delete[] syms_enc_bit_count;
	}

	static HuffmanTable optimize(const std::map<uint32_t, uint32_t> &sym_occ)	// <occurence count, symbol>
	{
		std::vector<SymProb> syms;
		syms.reserve(sym_occ.size());
		{
			auto end = sym_occ.rend();
			for (auto it = sym_occ.rbegin(); it != end; it++)
				syms.emplace_back(SymProb{it->first, it->second});
		}
		size_t min_i, min_j;
		size_t min_size = static_cast<size_t>(-1);
		for (size_t i = 4; i < syms.size(); i++)
			for (size_t j = 0; j < i; j++) {
				auto t = HuffmanTable(syms, i, j);
				if (t.max_bit_count <= 8) {
					auto size = t.model_size;
					if (size < min_size) {
						min_size = size;
						min_i = i;
						min_j = j;
					}
				}
			}
		if (min_size == static_cast<size_t>(-1))
			return HuffmanTable(syms, syms.size(), syms.size() - 1);
		else
			return HuffmanTable(syms, min_i, min_j);
	}

	uint8_t* encode(uint8_t *dst, const uint8_t *src, const size_t size) const
	{
		uint32_t csize = 0;
		uint32_t cbuf = 0;
		auto wc = [&](uint32_t size, uint32_t v) {
			cbuf += v << csize;
			csize += size;
			while (csize >= 8) {
				*dst++ = cbuf & 0xFF;
				cbuf >>= 8;
				csize -= 8;
			}
		};
		auto ws = [&](uint32_t count) {
			uint32_t acc = count;
			while (true) {
				uint32_t sm;
				for (sm = min(acc, max_sym_enc);; sm--)
					if (is_sym_enc[sm])
						break;
				wc(syms_enc_bit_count[sm], syms_enc[sm]);
				acc -= sm;
				if (acc == 0)
					break;
				wc(syms_enc_bit_count[0], syms_enc[0]);
			}
		};
		bool cur = false;
		size_t count = 0;
		auto src_max = src + size;
		for (auto p = src; p < src_max; p++)
			for (size_t i = 1; i < 1 << 8; i <<= 1) {
				auto c = (*p & i) != 0;
				if (c == cur)
					count++;
				else {
					ws(count);
					count = 1;
					cur = c;
				}
			}
		if (count > 0)
			ws(count);
		if (csize > 0)
			*dst++ = cbuf & 0xFF;
		return dst;
	}

	void decode(uint8_t *dst, const size_t dst_size, const uint8_t *src) const
	{
		uint32_t masks[9] {};
		for (size_t i = 1; i <= 8; i++)
			for (size_t j = 0; j < i; j++)
				masks[i] += 1 << j;

		uint32_t dsize = 0;
		uint32_t dbuf = 0;
		auto dst_max = dst + dst_size;
		auto wd = [&](uint32_t size, uint32_t v) {
			dbuf += v << dsize;
			dsize += size;
			while (dsize >= 8) {
				*dst++ = dbuf & 0xFF;
				dbuf >>= 8;
				dsize -= 8;
			}
		};
		auto wp = [&](uint32_t size, uint32_t v) {
			uint32_t its = size / 8;
			for (size_t i = 0; i < its; i++)
				wd(8, v);
			uint32_t md = size % 8;
			if (md)
				wd(md, v & masks[md]);
		};
		auto p = src;
		uint32_t bsize = 0;
		uint32_t buf = 0;
		bool state = false;
		while (true) {
			if (bsize < 8) {
				buf += *p++ << bsize;
				bsize += 8;
			}
			for (size_t i = 1; i <= 8; i++) {
				uint32_t v = (buf & masks[i]) + (1 << i);
				if (is_sym[v]) {
					wp(syms[v], state ? 0xFF : 0x00);
					bsize -= i;
					buf >>= i;
					state = !state;
					if (dst >= dst_max)
						return;
					break;
				}
			}
		}
	}

	bool validate(const uint8_t *buf, const size_t size, size_t &enc_size) const
	{
		std::vector<uint8_t> c;
		c.resize(size);
		{
			auto d = c.data();
			size_t s = encode(d, buf, size) - d;
			c.resize(s);
		}
		enc_size = c.size();
		std::vector<uint8_t> d;
		d.resize(size);
		decode(d.data(), size, c.data());
		return std::memcmp(buf, d.data(), size) == 0;
	}

	std::string exp(void) const
	{
		std::stringstream ss;
		ss << std::hex;
		auto pval = [&](auto val) -> void {
			ss << "0x" << static_cast<size_t>(val);
		};

		auto parr = [&](const char *type, auto &ptr, size_t size) -> void {
			ss << "new " << type << "[";
			pval(size);
			ss << "] {";
			auto sep = "";
			for (size_t i = 0; i < size; i++) {
				ss << sep;
				pval(ptr[i]);
				sep = ", ";
			}
			ss << "}";
		};

		ss << "static inline const auto dh = HuffmanTable{" << max_bit_count << ", ";
		parr("bool", is_sym, max_sym + 1);
		ss << ", ";
		parr("uint32_t", syms, max_sym + 1);
		ss << ", ";
		pval(max_sym);

		ss << ", ";
		parr("bool", is_sym_enc, max_sym_enc + 1);
		ss << ", ";
		parr("uint32_t", syms_enc, max_sym_enc + 1);
		ss << ", ";
		parr("uint8_t", syms_enc_bit_count, max_sym_enc + 1);
		ss << ", ";
		pval(max_sym_enc);
		ss << ", ";
		pval(model_size);
		ss << "};";

		return ss.str();
	}
};

static inline const auto dh = HuffmanTable{5, new bool[0x80] {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x0, 0x1, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, new uint32_t[0x80] {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x2, 0x0, 0x0, 0x1, 0x10, 0x0, 0x0, 0x30, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, 0x7f, new bool[0x31] {0x1, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1}, new uint32_t[0x31] {0x0, 0x5, 0x2, 0x3, 0xc, 0x0, 0x0, 0x0, 0x9, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1}, new uint8_t[0x31] {0x3, 0x3, 0x3, 0x2, 0x5, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x5, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4}, 0x30, 0x220a};

struct BlksBuf {
	struct Blk {
		uint8_t nibs[8];
		uint8_t nib_count = 0;
	};
	Blk *blks = new Blk[Enc::def().blk_count];
	uint8_t *dbpp = new uint8_t[Enc::dw * Enc::dh / 8];

	~BlksBuf(void)
	{
		delete[] blks;
		delete[] dbpp;
	}
};

struct pxd {
	double data[3];
	size_t i = 0;

	pxd(void)
	{
		for (size_t i = 0; i < 3; i++)
			data[i] = 0.0;
	}

	void inc(const px &p)
	{
		for (size_t i = 0; i < 3; i++)
			data[i] = (data[i] * static_cast<double>(i) + p.data[i]) / static_cast<double>(i + 1);
		i++;
	}

	px out(void) const
	{
		uint8_t d[3]{
			static_cast<uint8_t>(data[0]),
			static_cast<uint8_t>(data[1]),
			static_cast<uint8_t>(data[2])
		};
		return px(d);
	}
};

#ifdef _3DSSTREAM_HOST
template <size_t PoolSize, typename ...Args>
class Par {
	std::jthread m_pool[PoolSize];
	std::condition_variable m_cv;
	std::mutex m_mtx;
	std::optional<size_t> m_range;
	std::optional<std::tuple<Args...>> m_inv;

	std::condition_variable m_cv_compl;
	std::mutex m_mtx_compl;
	size_t m_compl;

	std::condition_variable m_cv_reset;
	std::mutex m_mtx_reset;

	template <typename Fn, size_t ...Idx>
	void apply_impl(Fn &&fn, size_t j, std::index_sequence<Idx...>)
	{
		fn(j, std::get<Idx>(*m_inv)...);
	}

public:
	template <typename Fun>
	Par(Fun &&fn)
	{
		for (size_t i = 0; i < PoolSize; i++)
			m_pool[i] = std::jthread([this, i, fn]() {
				while (true) {
					size_t range;
					{
						std::unique_lock l(m_mtx);
						m_cv.wait(l, [this, &range]() -> bool {
							if (m_range) {
								range = *m_range;
								return true;
							} else
								return false;
						});
					}
					size_t its = 0;
					for (size_t j = i; j < range; j += PoolSize, its++)
						apply_impl(fn, j, std::make_index_sequence<sizeof...(Args)>{});
					{
						std::lock_guard l(m_mtx_compl);
						m_compl += its;
					}
					m_cv_compl.notify_one();
					{
						std::unique_lock l(m_mtx_reset);
						m_cv_reset.wait(l, [this]() -> bool { return !m_range; });
					}
				}
			});
	}

	template <size_t Size>
	void dispatch(Args ...args)
	{
		{
			std::lock_guard l(m_mtx);
			m_compl = 0;
			m_range = Size;
			m_inv = std::tuple<Args...>(args...);
		}
		m_cv.notify_all();
		{
			std::unique_lock l(m_mtx_compl);
			m_cv_compl.wait(l, [this]() { return m_compl == Size; });
		}
		{
			std::lock_guard l(m_mtx_reset);
			m_range.reset();
		}
		m_cv_reset.notify_all();
	}
};

void cmp_pblk(size_t i, const uint8_t *glast, uint8_t *gcur, uint8_t *data, BlksBuf &buf)
{
	static constexpr auto e = Enc::def();
	static constexpr size_t vs = e.blk_size / 8;
	auto dbpp = buf.dbpp + i * e.h * 2;
	auto cb = buf.blks + i * e.h;
	auto last = glast + i * e.h * Enc::bfstride;
	auto cur = gcur + i * e.h * Enc::bfstride;
	auto last_base = glast;
	auto last_end = glast + e.blk_count * Enc::bfstride;
	for (size_t j = 0; j < e.h; j++, last += Enc::bfstride, cur += Enc::bfstride, cb++) {
		auto addr_blk = [data, i, j](size_t x, size_t y) {
			return data + ((j * e.blk_size + y) * Enc::dw + i * e.blk_size + x) * 3;
		};

		px cs[2] {
			addr_blk(e.blk_size / 4, e.blk_size / 4),
			addr_blk(e.blk_size * 3 / 4, e.blk_size * 3 / 4)
		};
		static_assert(Enc::bfstride == sizeof(cs), "Palette mismatch");

		for (size_t i = 0; i < 4; i++) {
			pxd pd[2];
			for (size_t i = 0; i < e.blk_size; i++)
				for (size_t j = 0; j < e.blk_size; j++) {
					auto c = px(addr_blk(i, j));
					if (c.dst(cs[0]) < c.dst(cs[1]))
						pd[0].inc(c);
					else
						pd[1].inc(c);
				}
			for (size_t i = 0; i < 2; i++)
				cs[i] = pd[i].out();
		}
		bool is_ref = false;
		uint8_t ref;

		{
			px pp_best[2] {
				addr_blk(e.blk_size / 4, e.blk_size / 4),
				addr_blk(e.blk_size * 3 / 4, e.blk_size * 3 / 4)
			};
			size_t pp_best_ndx;
			size_t pp_best_score = static_cast<size_t>(-1);
			for (size_t i = 0; i < 16; i += 2) {
				auto p = last + e.blk_off[i];
				if (!(p >= last_base && p < last_end))
					continue;
				px pp[2] {
					addr_blk(e.blk_size / 4, e.blk_size / 4),
					addr_blk(e.blk_size * 3 / 4, e.blk_size * 3 / 4)
				};
				std::memcpy(pp, p, sizeof(pp));
				size_t s = 0;
				for (size_t i = 0; i < e.blk_size; i++)
					for (size_t j = 0; j < e.blk_size; j++) {
						auto c = px(addr_blk(i, j));
						s += min(c.dst(pp[0]), c.dst(pp[1]));
					}
				if (s < pp_best_score) {
					pp_best_ndx = i;
					pp_best_score = s;
					std::memcpy(pp_best, pp, sizeof(pp_best));
				}
			}
			pp_best_score /= e.blk_px_count;
			if (pp_best_score < 64 * 64) {
				is_ref = true;
				ref = 1 + pp_best_ndx;
				std::memcpy(cs, pp_best, sizeof(cs));
			}
		}

		if constexpr (e.blk_size == 4) {
			for (size_t i = 0; i < e.blk_size / 2; i++) {
				uint8_t v = 0;
				for (size_t k = 0; k < 4; k++) {
					auto c = px(addr_blk(i * 2, k));
					if (c.dst(cs[0]) > c.dst(cs[1]))
						v |= (1 << k);
				}
				for (size_t k = 4; k < 8; k++) {
					auto c = px(addr_blk(i * 2 + 1, k - 4));
					if (c.dst(cs[0]) > c.dst(cs[1]))
						v |= (1 << k);
				}
				*dbpp++ = v;
			}
		} else {
			for (size_t i = 0; i < e.blk_size; i++)
				for (size_t j = 0; j < vs; j++) {
					uint8_t v = 0;
					for (size_t k = 0; k < 8; k++) {
						auto c = px(addr_blk(i, j * 8 + k));
						if (c.dst(cs[0]) > c.dst(cs[1]))
							v |= (1 << k);
					}
					*dbpp++ = v;
				}
		}
		std::memcpy(cur, cs, sizeof(cs));
		*cb = BlksBuf::Blk();
		auto w_nib = [&](uint8_t v) {
			cb->nibs[cb->nib_count++] = v;
		};
		if (is_ref)
			w_nib(ref);
		else
			for (size_t i = 0; i < 2; i++) {
				auto c = cs[i].cf1r5g5b5(0);
				for (size_t i = 0; i < 4; i++) {
					size_t o = i * 4;
					w_nib((c & (0x0F << o)) >> o);
				}
			}
	}
};
using ParCmp = Par<16, const uint8_t *, uint8_t *, uint8_t *, BlksBuf &>;
static auto enc_par = ParCmp(cmp_pblk);
#endif

class Img
{
	size_t m_w;
	size_t m_h;
	uint8_t *m_data;

public:
	static size_t computeStride(size_t w)
	{
		return align_up(w * 3, 4);
	}

	uint8_t* get_data(void)
	{
		return m_data;
	}

	static Img create(void)
	{
		return Img(Enc::dw, Enc::dh);
	}

#ifdef _WINGDI_
	Img(const char *filename)
	{
		FILE *f = fopen(filename, "rb");
		if (f == nullptr)
			throw std::runtime_error(filename);
		fseek(f, 0x0A, SEEK_SET);
		uint32_t off;
		assert(fread(&off, sizeof(off), 1, f) == 1);
		fseek(f, 0x0E, SEEK_SET);
		BITMAPINFOHEADER h;
		assert(fread(&h, sizeof(h), 1, f) == 1);
		m_w = h.biWidth;
		m_h = h.biHeight;
		m_data = new uint8_t[m_w * m_h * 3];
		if (h.biBitCount != 24)
			throw std::runtime_error("BPP must be 24");

		size_t s = computeStride(m_w);
		size_t ds = m_w * 3;
		for (size_t i = 0; i < m_h; i++) {
			fseek(f, off + s * i, SEEK_SET);
			assert(fread(m_data + ds * i, ds, 1, f) == 1);
		}
		fclose(f);
		//std::printf("%zu, %zu\n", m_w, m_h);
	}

	void load(const uint8_t *bmpdata)
	{
		size_t s = computeStride(m_w);
		size_t ds = m_w * 3;
		for (size_t i = 0; i < m_h; i++)
			std::memcpy(m_data + ds * i, bmpdata + s * i, ds);
	}
#endif
	Img(size_t w, size_t h) :
		m_w(w),
		m_h(h),
		m_data(new uint8_t[m_w * m_h * 3])
	{
	}
	~Img(void)
	{
		delete[] m_data;
	}

#ifdef _WINGDI_
	void out(const char *filename)
	{
		FILE *f = fopen(filename, "wb");
		if (f == nullptr)
			throw std::runtime_error(filename);

		auto s = computeStride(m_w);
		assert(fwrite("BM", 2, 1, f) == 1);
		{
			uint32_t b = 0x0E + sizeof(BITMAPINFOHEADER) + s * m_h;
			assert(fwrite(&b, sizeof(uint32_t), 1, f) == 1);
			b = 0;
			assert(fwrite(&b, sizeof(uint16_t), 1, f) == 1);
			assert(fwrite(&b, sizeof(uint16_t), 1, f) == 1);
			b = 0x0E + sizeof(BITMAPINFOHEADER);
			assert(fwrite(&b, sizeof(uint32_t), 1, f) == 1);
		}

		BITMAPINFOHEADER hi{
			sizeof(hi),	// biSize
			static_cast<int32_t>(m_w),	// biWidth
			static_cast<int32_t>(m_h),	// biWidth
			1,	// biPlanes
			24,	// biBitCount
			BI_RGB,	// biCompression
			static_cast<uint32_t>(s * m_h),	// biSizeImage
			1,	// biXPelsPerMeter
			1,	// biYPelsPerMeter
			0,	// biClrUsed
			0	// biClrImportant
		};
		assert(fwrite(&hi, sizeof(hi), 1, f) == 1);

		size_t ds = m_w * 3;
		uint8_t row[s] {};
		for (size_t i = 0; i < m_h; i++) {
			std::memcpy(row, m_data + ds * i, ds);
			assert(fwrite(row, s, 1, f) == 1);
		}
		fclose(f);
	}
#endif

	void sample(Img &dst)
	{
		static auto nr = [](size_t i, size_t s) {
			return static_cast<double>(i) / static_cast<double>(s - 1);
		};
		static auto dnr = [](double v, size_t s) {
			return static_cast<size_t>(v * static_cast<double>(s - 1));
		};
		static auto cv = [](size_t i, size_t ss, size_t ds) {
			return dnr(nr(i, ss), ds);
		};
		for (size_t i = 0; i < dst.m_h; i++)
			for (size_t j = 0; j < dst.m_w; j++)
				reinterpret_cast<px*>(dst.m_data)[i * dst.m_w + j] =
					reinterpret_cast<px*>(m_data)[cv(i, dst.m_h, m_h) * m_w + cv(j, dst.m_w, m_w)];
	}

	static constexpr auto de = Enc::def();
	//static_assert(de.blk_count % 8 == 0, "Must have blk_count multiple of 8");

	uint8_t* alloc_cmp(const Enc &e)
	{
		size_t size = e.blk_count * 4 +	// up to 32 bits per block
			m_w * m_h / 8 +	// 1 bpp
			2;	// 16 bits size included (only for frame size retrieval, not needed for decoding)
		return new uint8_t[size];
	}

	uint8_t *alloc_blk(const Enc &e)
	{
		size_t size = e.blk_count * Enc::bfstride;
		auto res = new uint8_t[size];
		std::memset(res, 0, size);
		return res;
	}

	#ifdef _3DSSTREAM_HOST
	template <Enc e>
	uint16_t cmp(const uint8_t *glast, uint8_t *gcur, uint8_t *gdst, size_t frame_ndx, BlksBuf &buf)	// frame cannot exceed 64 KiB
	{
		static constexpr size_t dbpp_size = Enc::dw * Enc::dh / 8;
		auto dst_base = gdst;
		gdst += 2;
		enc_par.dispatch<e.w>(glast, gcur, m_data, buf);
		bool ch = false;
		auto w_nib = [&](uint8_t v) {
			if (ch) {
				*gdst++ += v << 4;
				ch = false;
			} else {
				*gdst = v;
				ch = true;
			}
		};
		for (size_t i = 0; i < e.blk_count; i++) {
			auto &c = buf.blks[i];
			for (size_t j = 0; j < c.nib_count; j++)
				w_nib(c.nibs[j]);
		}
		if (ch)
			gdst++;
		gdst = dh.encode(gdst, buf.dbpp, dbpp_size);
		if (frame_ndx % 60 == 0 && false) {
			/*std::map<size_t, size_t> m;
			bool cur = false;
			size_t count = 0;
			for (auto p = buf.dbpp; p < buf.dbpp + dbpp_size; p++)
				for (size_t i = 1; i < 1 << 8; i <<= 1) {
					auto c = (*p & i) != 0;
					if (c == cur) {
						count++;
					} else {
						m[count]++;
						count = 1;
						cur = c;
					}
				}
			std::map<uint32_t, uint32_t> mr;
			for (auto &p : m)
				mr[p.second] = p.first;
			auto table = HuffmanTable::optimize(mr);
			std::printf("%s\n", table.exp().c_str());*/

			auto &table = dh;

			size_t enc_size;
			auto valid = table.validate(buf.dbpp, dbpp_size, enc_size);
			std::printf("BEST COMPR: %zu (raw: %zu); max sym: %u, valid: %d\n",
				enc_size,
				dbpp_size,
				table.max_sym,
				valid);
		}
		size_t size = gdst - dst_base;
		*reinterpret_cast<uint16_t*>(dst_base) = size - 2;
		return size;
	}
	#endif

	void flip(uint8_t *dst)
	{
		for (size_t i = 0; i < m_h; i++)
			for (size_t j = 0; j < m_w; j++)
				for (size_t k = 0; k < 3; k++)
					dst[(j * m_h + i) * 3 + k] = m_data[(i * m_w + j) * 3 + k];
	}

	template <Enc e>
	static void dcmp(const uint8_t *c, const uint8_t *last, uint8_t *cur, uint8_t *dst, BlksBuf &buf)
	{
		{
			static constexpr auto upck_rbg = [](uint16_t src, uint8_t *&dst)
			{
				*dst++ = (src & 0x003E) << 2;
				*dst++ = (src & 0x07C0) >> 3;
				*dst++ = (src & 0xF800) >> 8;
			};

			auto cr = cur;
			uint8_t b = *c++;
			bool bh = false;
			const uint8_t *mx = last + e.blk_count * Enc::bfstride;
			for (; last < mx; last += Enc::bfstride) {
				if (b & 1) {	// blk in last frame
					auto l = last + e.blk_off[b & 0x0F];
					for (size_t i = 0; i < Enc::bfstride; i++)
						*cr++ = *l++;
					if (bh) {
						bh = false;
						b = *c++;
					} else {
						bh = true;
						b >>= 4;
					}
				} else {	// new blk data
					if (bh) {
						uint32_t v = *reinterpret_cast<const uint32_t*>(c);
						upck_rbg(b + ((v & 0x00000FFF) << 4), cr);
						upck_rbg(b + ((v & 0x0FFFF000) >> 12), cr);
						c += 4;
						b = v >> 28;
					} else {
						upck_rbg(*reinterpret_cast<const uint16_t*>(c++ - 1), cr);
						upck_rbg(*reinterpret_cast<const uint16_t*>(c), cr);
						c += 2;
						b = *c++;
					}
				}
			}
			// if b at start of byte, rewind c to match it
			// otherwise if b is high, skip it
			if (!bh)
				c--;
		}
		static constexpr size_t dbpp_size = Enc::dw * Enc::dh / 8;
		dh.decode(buf.dbpp, dbpp_size, c);
		c = buf.dbpp;

		{
			static constexpr size_t fbbstride_i = e.h * ((e.blk_size - 1) * e.blk_size) * sizeof(px);
			static constexpr size_t fbbstride_j = e.blk_size * sizeof(px);
			static constexpr size_t vs = e.blk_size / 8;
			// iterate through blocks, block palette and framebuffer wise
			auto b = cur, fb = dst;
			for (size_t i = 0; i < e.w; i++, fb += fbbstride_i)
				for (size_t j = 0; j < e.h; j++, b += Enc::bfstride, fb += fbbstride_j) {
					auto f = fb;
					if constexpr (e.blk_size == 4)
						for (size_t i = 0; i < e.blk_size / 2; i++, f += Enc::dh * sizeof(px)) {
							auto fc = f;
							auto v = *c++;
							for (size_t k = 1; k < (1 << 4); k <<= 1) {
								auto p = b + (v & k ? sizeof(px) : 0);
								for (size_t i = 0; i < 3; i++)
									*fc++ = *p++;
							}
							f += Enc::dh * sizeof(px);
							fc = f;
							for (size_t k = 1 << 4; k < (1 << 8); k <<= 1) {
								auto p = b + (v & k ? sizeof(px) : 0);
								for (size_t i = 0; i < 3; i++)
									*fc++ = *p++;
							}
						}
					else
						for (size_t i = 0; i < e.blk_size; i++, f += Enc::dh * sizeof(px)) {
							auto fc = f;
							for (size_t j = 0; j < vs; j++) {
								auto v = *c++;
								static constexpr size_t mx = 1 << 8;
								for (size_t k = 1; k < mx; k <<= 1) {
									auto p = b + (v & k ? sizeof(px) : 0);
									for (size_t i = 0; i < 3; i++)
										*fc++ = *p++;
								}
							}
						}
				}
		}
	}
};