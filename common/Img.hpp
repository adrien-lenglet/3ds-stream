#pragma once

#include <cstdint>
#include <cstring>

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
			size_t d = data[i] - other.data[i];
			res += d * d;
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
	static constexpr size_t dblk_size = 8;
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
		return new uint8_t[size];
	}

	template <Enc e>
	uint16_t cmp(const uint8_t *last, uint8_t *cur, uint8_t *dst)	// frame cannot exceed 64 KiB
	{
		static constexpr size_t dbpp_size = Enc::dw * Enc::dh / 8;
		auto dst_base = dst;
		dst += 2;
		auto dbpp_base = dst + e.blk_count * 4;
		auto dbpp = dbpp_base;
		bool ch = false;
		static constexpr size_t vs = e.blk_size / 8;
		auto data = m_data;
		auto w_nib = [&](uint8_t v) {
			if (ch) {
				*dst++ += v << 4;
				ch = false;
			} else {
				*dst = v;
				ch = true;
			}
		};
		auto last_base = last;
		auto last_end = last + e.blk_count * Enc::bfstride;
		for (size_t i = 0; i < e.w; i++)
			for (size_t j = 0; j < e.h; j++, last += Enc::bfstride, cur += Enc::bfstride) {
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
					if (pp_best_score < 16) {
						is_ref = true;
						ref = 1 + pp_best_ndx;
						std::memcpy(cs, pp_best, sizeof(cs));
					}
				}

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
				std::memcpy(cur, cs, sizeof(cs));
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
		if (ch)
			dst++;
		std::memmove(dst, dbpp_base, dbpp_size);
		dst += dbpp_size;
		size_t size = dst - dst_base;
		*reinterpret_cast<uint16_t*>(dst_base) = size - 2;
		return size;
	}

	void flip(uint8_t *dst)
	{
		for (size_t i = 0; i < m_h; i++)
			for (size_t j = 0; j < m_w; j++)
				for (size_t k = 0; k < 3; k++)
					dst[(j * m_h + i) * 3 + k] = m_data[(i * m_w + j) * 3 + k];
	}

	template <Enc e>
	static void dcmp(const uint8_t *c, const uint8_t *last, uint8_t *cur, uint8_t *dst)
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

		{
			static constexpr size_t fbbstride_i = e.h * ((e.blk_size - 1) * e.blk_size) * sizeof(px);
			static constexpr size_t fbbstride_j = e.blk_size * sizeof(px);
			static constexpr size_t vs = e.blk_size / 8;
			// iterate through blocks, block palette and framebuffer wise
			auto b = cur, fb = dst;
			for (size_t i = 0; i < e.w; i++, fb += fbbstride_i)
				for (size_t j = 0; j < e.h; j++, b += Enc::bfstride, fb += fbbstride_j) {
					auto f = fb;
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