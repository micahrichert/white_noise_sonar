#if (USE_BUILTIN)
    void popcount_setup()
    {
    }

    inline unsigned int popcount(uint32_t i)
    {
        // some instruction sets have a dedicated instruction for popcount, in which case the builtin will be fastest, 
        // but if there isn't a special instruction then __builtin_popcount() is quite slow.
        return __builtin_popcount(i);
    }
#endif


inline unsigned int parallel_popcount(uint32_t i)
{
  //Parallel binary bit add
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

#if (USE_PARALLEL_BITS)
    void popcount_setup()
    {
    }

    // a little slower than using lookup table but doesn't require any memory
    inline unsigned int popcount(uint32_t i)
    {
        return parallel_popcount(i);
    }
#endif

#if (USE_LOOKUP)
    // uses 64K of memory but is fastest implementation
    // don't make it const and/or static because it will go into flash memory which then makes it slower than PARALLEL_BITS
    uint8_t wordbits[65536];// very fast but uses 64K of memory
    
    void popcount_setup()
    {
        for (uint32_t i=0; i<65536; i++) wordbits[i] = parallel_popcount(i);
    }

    inline unsigned int popcount(uint32_t i)
    {
        return (wordbits[i&0xFFFF] + wordbits[i>>16]);
    }
#endif

