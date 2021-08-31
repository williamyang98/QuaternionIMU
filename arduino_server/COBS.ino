// convert buffer to COBS encoding (using zero byte delimiting)
int cobsEncode(const uint8_t *data, int n, uint8_t *buf)
{
  uint8_t *encode = buf; // Encoded byte pointer
  uint8_t *codep = encode++; // Output code pointer
  uint8_t code = 1; // Code value

  for (const uint8_t *b = data; n--; ++b)
  {
    if (*b) // Byte not zero, write it
      *encode++ = *b, ++code;

    if (!*b || code == 0xff) // Input is zero or block completed, restart
    {
      *codep = code, code = 1, codep = encode;
      if (!*b || n)
        ++encode;
    }
  }
  *codep = code; // Write final code value

  return (int)(encode - buf);
}

// decode a COBS encoded (zero byte delimited) packet into original form
int cobsDecode(const uint8_t *buf, int len, uint8_t *data)
{
  const uint8_t *encoded = buf; // Encoded input byte pointer
  uint8_t *decoded = data; // Decoded output byte pointer

  for (uint8_t code = 0xff, block = 0; encoded < buf + len; --block)
  {
    if (block) // Decode block byte
      *decoded++ = *encoded++;
    else
    {
      if (code != 0xff) // Encoded zero, write it
        *decoded++ = 0;
      block = code = *encoded++; // Next block length
      if (code == 0x00) // Delimiter code found
        break;
    }
  }

  return (int)(decoded-data);
}
