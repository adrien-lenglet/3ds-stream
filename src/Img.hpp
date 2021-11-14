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
		return Img(400, 240);
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

	struct Enc {
		size_t blk_size;
		size_t blk_stride;
		size_t blk_px_count;

		Enc(size_t blk_size) :
			blk_size(blk_size),
			blk_stride(sizeof(px) * 2 + (blk_size * blk_size) / 8),
			blk_px_count(blk_size * blk_size)
		{
		}

		static Enc create(void)
		{
			return Enc(16);
		}
	};

	size_t blk_count(const Enc &e)
	{
		return (m_w / e.blk_size) * (m_h / e.blk_size);
	}

	size_t cmp_size(const Enc &e)
	{
		return blk_count(e) * e.blk_stride;
	}

	uint8_t* alloc_cmp(const Enc &e)
	{
		return new uint8_t[cmp_size(e)];
	}

	void cmp(const Enc &e, uint8_t *dst)
	{
		size_t w = m_w / e.blk_size, h = m_h / e.blk_size;
		size_t blk_stride = e.blk_stride;
		size_t hs = e.blk_size / 8;
		for (size_t i = 0; i < h; i++)
			for (size_t j = 0; j < w; j++) {
				auto addr_blk = [&](size_t y, size_t x) {
					return m_data + ((i * e.blk_size + y) * m_w + j * e.blk_size + x) * 3;
				};

				px cs[2] {
					addr_blk(e.blk_size / 4, e.blk_size / 4),
					addr_blk(e.blk_size * 3 / 4, e.blk_size * 3 / 4)
				};

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
				size_t coff = (i * w + j) * blk_stride;
				for (size_t i = 0; i < e.blk_size; i++)
					for (size_t j = 0; j < hs; j++) {
						uint8_t v = 0;
						for (size_t k = 0; k < 8; k++) {
							auto c = px(addr_blk(i, j * 8 + k));
							if (c.dst(cs[0]) > c.dst(cs[1]))
								v |= (1 << k);
						}
						dst[coff + sizeof(cs) + i * hs + j] = v;
					}
				std::memcpy(dst + coff, cs, sizeof(cs));
			}
	}

	void flip(uint8_t *dst)
	{
		for (size_t i = 0; i < m_h; i++)
			for (size_t j = 0; j < m_w; j++)
				for (size_t k = 0; k < 3; k++)
					dst[(j * m_h + i) * 3 + k] = m_data[(i * m_w + j) * 3 + k];
	}

	void dcmp(const Enc &e, const uint8_t *c)
	{
		size_t w = m_w / e.blk_size, h = m_h / e.blk_size;
		size_t blk_stride = e.blk_stride;
		size_t hs = e.blk_size / 8;
		for (size_t i = 0; i < h; i++)
			for (size_t j = 0; j < w; j++) {
				auto addr_blk = [&](size_t y, size_t x) {
					return m_data + ((i * e.blk_size + y) * m_w + j * e.blk_size + x) * 3;
				};
				size_t coff = (i * w + j) * blk_stride;
				for (size_t i = 0; i < e.blk_size; i++)
					for (size_t j = 0; j < hs; j++) {
						uint8_t v = c[coff + sizeof(px) * 2 + i * hs + j];
						for (size_t k = 0; k < 8; k++) {
							auto *s = c + coff + (v & (1 << k) ? sizeof(px) : 0);
							auto c = addr_blk(i, j * 8 + k);
							for (size_t i = 0; i < 3; i++)
								c[i] = s[i];
						}
					}
			}
	}
};