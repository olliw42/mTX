/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <math.h>
#include <stdio.h>
#include <limits.h>
#include "opentx.h"
#include "strhelpers.h"

#if defined(SIMU)
display_t displayBuf[DISPLAY_BUFFER_SIZE];
#endif

uint16_t lcdColorTable[LCD_COLOR_COUNT];

coord_t lcdNextPos;

uint8_t getMappedChar(uint8_t c)
{
  uint8_t result;
  if (c == 0)
    result = 0;
#if defined(TRANSLATIONS_FR)
  else if (c >= 0x80 && c <= 0x84) {
    result = 115 + c - 0x80;
  }
#elif defined(TRANSLATIONS_DE)
  else if (c >= 0x80 && c <= 0x86) {
    result = 120 + c - 0x80;
  }
#elif defined(TRANSLATIONS_CZ)
  else if (c >= 0x80 && c <= 0x80+29) {
    result = 127 + c - 0x80;
  }
#elif defined(TRANSLATIONS_ES)
  else if (c >= 0x80 && c <= 0x80+12) {
    result = 157 + c - 0x80;
  }
#elif defined(TRANSLATIONS_FI) || defined(TRANSLATIONS_SE)
  else if (c >= 0x80 && c <= 0x85) {
    result = 169 + c - 0x80;
  }
#elif defined(TRANSLATIONS_IT)
  else if (c >= 0x80 && c <= 0x81) {
    result = 175 + c - 0x80;
  }
#elif defined(TRANSLATIONS_PL)
  else if (c >= 0x80 && c <= 0x80+17) {
    result = 177 + c - 0x80;
  }
#elif defined(TRANSLATIONS_PT)
  else if (c >= 0x80 && c <= 0x80+21) {
    result = 195 + c - 0x80;
  }
#endif
  else if (c < 0xC0)
    result = c - 0x20;
  else
    result = c - 0xC0 + 96;
    // TRACE("getMappedChar '%c' (%d) = %d", c, c, result);
  return result;
}

int getFontPatternWidth(const uint16_t * spec, uint8_t index)
{
  return spec[index+1] - spec[index];
}

int getCharWidth(uint8_t c, const uint16_t * spec)
{
  return getFontPatternWidth(spec, getMappedChar(c));
}

void lcdPutFontPattern(coord_t x, coord_t y, const uint8_t * font, const uint16_t * spec, int index, LcdFlags flags)
{
  coord_t offset = spec[index];
  coord_t width = spec[index+1] - offset;
  if (width > 0) lcdDrawBitmapPattern(x, y, font, flags, offset, width);
  lcdNextPos = x + width;
}

void lcdDrawChar(coord_t x, coord_t y, char c, LcdFlags flags)
{
  uint32_t fontindex = FONTINDEX(flags);
  const unsigned char * font = fontsTable[fontindex];
  const uint16_t * fontspecs = fontspecsTable[fontindex];
  lcdPutFontPattern(x, y, font, fontspecs, getMappedChar(c), flags);
}

uint8_t getStringInfo(const char *s)
{
  uint8_t result = 0;
  for (int i=0; s[i]; i++) {
    result += s[i];
  }
  return result;
}

uint8_t getFontHeight(LcdFlags flags)
{
  static const uint8_t heightTable[16] = { 9, 13, 16, 24, 32, 64, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };
  return heightTable[FONTINDEX(flags)];
}

int getTextWidth(const char * s, int len, LcdFlags flags)
{
  const uint16_t * specs = fontspecsTable[FONTINDEX(flags)];

  int result = 0;
  for (int i=0; len==0 || i<len; ++i) {

#if !defined(BOOT)
    char c = (flags & ZCHAR) ? zchar2char(*s) : *s;
#else
    char c = *s;
#endif
    if (c == '\0')
      break;
    result += getCharWidth(c, specs);
    ++s;
  }
  return result;
}

void lcdDrawTextAtIndex(coord_t x, coord_t y, const char * s, uint8_t idx, LcdFlags flags)
{
  uint8_t length;
  length = *(s++);
  lcdDrawSizedText(x, y, s+length*idx, length, flags & ~ZCHAR);
}

void lcdDrawHexNumber(coord_t x, coord_t y, uint32_t val, LcdFlags flags)
{
  char s[5];
  for (int i=0; i<4; i++) {
    char c = val & 0xf;
    s[3-i] = c>9 ? c+'A'-10 : c+'0';
    val >>= 4;
  }
  s[4] = '\0';
  lcdDrawText(x, y, s, flags);
}

void lcdDrawNumber(coord_t x, coord_t y, int32_t val, LcdFlags flags, uint8_t len, const char * prefix, const char * suffix)
{
  char str[48+1]; // max=16 for the prefix, 16 chars for the number, 16 chars for the suffix
  char *s = str+32;
  *s = '\0';
  int idx = 0;
  int mode = MODE(flags);
  bool neg = false;

  if (val == INT_MAX) {
    flags &= ~(LEADING0 | PREC1 | PREC2);
    lcdDrawText(x, y, "INT_MAX", flags);
    return;
  }

  if (val < 0) {
    if (val == INT_MIN) {
      flags &= ~(LEADING0 | PREC1 | PREC2);
      lcdDrawText(x, y, "INT_MIN", flags);
      return;
    }
    val = -val;
    neg = true;
  }
  do {
    *--s = '0' + (val % 10);
    ++idx;
    val /= 10;
    if (mode!=0 && idx==mode) {
      mode = 0;
      *--s = '.';
      if (val==0)
        *--s = '0';
    }
  } while (val!=0 || mode>0 || (mode==MODE(LEADING0) && idx<len));
  if (neg) *--s = '-';

  // TODO needs check on all string lengths ...
  if (prefix) {
    int len = strlen(prefix);
    if (len <= 16) {
      s -= len;
      strncpy(s, prefix, len);
    }
  }
  if (suffix) {
    strncpy(&str[32], suffix, 16);
  }
  flags &= ~LEADING0;
  lcdDrawText(x, y, s, flags);
}

void lcdDrawLine(coord_t x1, coord_t y1, coord_t x2, coord_t y2, uint8_t pat, LcdFlags att)
{
  int dx = x2-x1;      /* the horizontal distance of the line */
  int dy = y2-y1;      /* the vertical distance of the line */
  int dxabs = abs(dx);
  int dyabs = abs(dy);
  int sdx = sgn(dx);
  int sdy = sgn(dy);
  int x = dyabs>>1;
  int y = dxabs>>1;
  int px = x1;
  int py = y1;

  if (dxabs >= dyabs) {
    /* the line is more horizontal than vertical */
    for (int i=0; i<=dxabs; i++) {
      if ((1<<(px%8)) & pat) {
        lcdDrawPoint(px, py, att);
      }
      y += dyabs;
      if (y>=dxabs) {
        y -= dxabs;
        py += sdy;
      }
      px += sdx;
    }
  }
  else {
    /* the line is more vertical than horizontal */
    for (int i=0; i<=dyabs; i++) {
      if ((1<<(py%8)) & pat) {
        lcdDrawPoint(px, py, att);
      }
      x += dxabs;
      if (x >= dyabs) {
        x -= dyabs;
        px += sdx;
      }
      py += sdy;
    }
  }
}

#if !defined(BOOT)
void drawRtcTime(coord_t x, coord_t y, LcdFlags flags)
{
  drawTimer(x, y, getValue(MIXSRC_TX_TIME), flags);
}

void drawTimer(coord_t x, coord_t y, int32_t tme, LcdFlags flags)
{
  char str[LEN_TIMER_STRING];
  getTimerString(str, tme, (flags & TIMEHOUR) != 0);
  lcdDrawText(x, y, str, flags);
}

void putsStickName(coord_t x, coord_t y, uint8_t idx, LcdFlags att)
{
  uint8_t length = STR_VSRCRAW[0];
  lcdDrawSizedText(x, y, STR_VSRCRAW+2+length*(idx+1), length-1, att);
}

void drawSource(coord_t x, coord_t y, mixsrc_t idx, LcdFlags flags)
{
  char s[16];
  getSourceString(s, idx);
  lcdDrawText(x, y, s, flags);
}

void putsChnLetter(coord_t x, coord_t y, uint8_t idx, LcdFlags att)
{
  lcdDrawTextAtIndex(x, y, STR_RETA123, idx-1, att);
}

void putsModelName(coord_t x, coord_t y, char * name, uint8_t id, LcdFlags att)
{
  uint8_t len = sizeof(g_model.header.name);
  while (len>0 && !name[len-1]) --len;
  if (len==0) {
    drawStringWithIndex(x, y, STR_MODEL, id+1, att|LEADING0);
  }
  else {
    lcdDrawSizedText(x, y, name, sizeof(g_model.header.name), ZCHAR|att);
  }
}

void drawSwitch(coord_t x, coord_t y, swsrc_t idx, LcdFlags flags, bool autoBold)
{
  char s[8];
  getSwitchPositionName(s, idx);
  if (autoBold && idx != SWSRC_NONE && getSwitch(idx))
    flags |= BOLD;
  lcdDrawText(x, y, s, flags);
}

void drawCurveName(coord_t x, coord_t y, int8_t idx, LcdFlags flags)
{
  char s[8];
  getCurveString(s, idx);
  lcdDrawText(x, y, s, flags);
}

void drawTimerMode(coord_t x, coord_t y, swsrc_t mode, LcdFlags att)
{
  if (mode >= 0) {
    if (mode < TMRMODE_COUNT) {
      lcdDrawTextAtIndex(x, y, STR_VTMRMODES, mode, att);
      return;
    }
    else {
      mode -= (TMRMODE_COUNT-1);
    }
  }
  drawSwitch(x, y, mode, att);
}

void drawTrimMode(coord_t x, coord_t y, uint8_t phase, uint8_t idx, LcdFlags att)
{
  trim_t v = getRawTrimValue(phase, idx);
  unsigned int mode = v.mode;
  unsigned int p = mode >> 1;

  if (mode == TRIM_MODE_NONE) {
    lcdDrawText(x, y, "--", att);
  }
  else {
    char s[2];
    s[0] = (mode % 2 == 0) ? ':' : '+';
    s[1] = '0'+p;
    lcdDrawSizedText(x, y, s, 2, att);
  }
}


void drawDate(coord_t x, coord_t y, TelemetryItem & telemetryItem, LcdFlags att)
{
  // TODO
  if (att & DBLSIZE) {
    x -= 42;
    att &= ~FONTSIZE_MASK;
    lcdDrawNumber(x, y, telemetryItem.datetime.day, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos-1, y, '-', att);
    lcdDrawNumber(lcdNextPos-1, y, telemetryItem.datetime.month, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos-1, y, '-', att);
    lcdDrawNumber(lcdNextPos-1, y, telemetryItem.datetime.year-2000, att|LEADING0|LEFT);
    y += FH;
    lcdDrawNumber(x, y, telemetryItem.datetime.hour, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos, y, ':', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.min, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos, y, ':', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.sec, att|LEADING0|LEFT, 2);
  }
  else {
    lcdDrawNumber(x, y, telemetryItem.datetime.day, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos-1, y, '-', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.month, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos-1, y, '-', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.year-2000, att|LEADING0|LEFT);
    lcdDrawNumber(lcdNextPos+11, y, telemetryItem.datetime.hour, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos, y, ':', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.min, att|LEADING0|LEFT, 2);
    lcdDrawChar(lcdNextPos, y, ':', att);
    lcdDrawNumber(lcdNextPos, y, telemetryItem.datetime.sec, att|LEADING0|LEFT, 2);
  }
}

void drawGPSCoord(coord_t x, coord_t y, int32_t value, const char * direction, LcdFlags flags, bool seconds=true)
{
  char s[32];
  uint32_t absvalue = abs(value);
  char * tmp = strAppendUnsigned(s, absvalue / 1000000);
  *tmp++ = '@';
  absvalue = absvalue % 1000000;
  absvalue *= 60;
  if (g_eeGeneral.gpsFormat == 0 || !seconds) {
    tmp = strAppendUnsigned(tmp, absvalue / 1000000, 2);
    *tmp++ = '\'';
    if (seconds) {
      absvalue %= 1000000;
      absvalue *= 60;
      absvalue /= 100000;
      tmp = strAppendUnsigned(tmp, absvalue / 10);
      *tmp++ = '.';
      tmp = strAppendUnsigned(tmp, absvalue % 10);
      *tmp++ = '"';
    }
  }
  else {
    tmp = strAppendUnsigned(tmp, absvalue / 1000000, 2);
    *tmp++ = '.';
    absvalue /= 1000;
    tmp = strAppendUnsigned(tmp, absvalue, 3);
  }
  *tmp++ = direction[value>=0 ? 0 : 1];
  *tmp = '\0';
  lcdDrawText(x, y, s, flags);
}

void drawGPSPosition(coord_t x, coord_t y, int32_t longitude, int32_t latitude, LcdFlags flags)
{
  if (flags & EXPANDED) {
    drawGPSCoord(x, y, latitude, "NS", flags, true);
    drawGPSCoord(x, y + FH, longitude, "EW", flags, true);
  }
  else {
    drawGPSCoord(x, y, latitude, "NS", flags, false);
    drawGPSCoord(lcdNextPos+5, y, longitude, "EW", flags, false);
  }
}

void drawGPSSensorValue(coord_t x, coord_t y, TelemetryItem & telemetryItem, LcdFlags flags)
{
  drawGPSPosition(x, y, telemetryItem.gps.longitude, telemetryItem.gps.latitude, flags);
}
#endif

void lcdSetContrast()
{
  lcdSetRefVolt(g_eeGeneral.contrast);
}

void lcdDrawPoint(coord_t x, coord_t y, LcdFlags att)
{
  display_t * p = PIXEL_PTR(x, y);
  display_t color = lcdColorTable[COLOR_IDX(att)];
#if defined(PCBX10)
  if (p >= displayBuf && x < LCD_W) {  // x10 screen is reversed, so overflow are happening the other way around
#else
  if (p < DISPLAY_END) {
#endif
    *p = color;
  }
}

void lcdDrawBlackOverlay()
{
  lcdDrawFilledRect(0, 0, LCD_W, LCD_H, SOLID, OVERLAY_COLOR | OPACITY(8));
}

#if defined(MULTIMODULE)
void lcdDrawMultiProtocolString(coord_t x, coord_t y, uint8_t moduleIdx, uint8_t protocol, LcdFlags flags)
{
  MultiModuleStatus & status = getMultiModuleStatus(moduleIdx);
  if (status.protocolName[0] && status.isValid()) {
    lcdDrawText(x, y, status.protocolName, flags);
  }
  else if (protocol <= MODULE_SUBTYPE_MULTI_LAST) {
    lcdDrawTextAtIndex(x, y, STR_MULTI_PROTOCOLS, protocol, flags);
  }
  else {
    lcdDrawNumber(x, y, protocol + 3, flags); // Convert because of OpenTX FrSky fidling (OpenTX protocol tables and Multiprotocol tables don't match)
  }
}

void lcdDrawMultiSubProtocolString(coord_t x, coord_t y, uint8_t moduleIdx, uint8_t subType, LcdFlags flags)
{
  MultiModuleStatus & status = getMultiModuleStatus(moduleIdx);
  const mm_protocol_definition * pdef = getMultiProtocolDefinition(g_model.moduleData[moduleIdx].getMultiProtocol());

  if (status.protocolName[0] && status.isValid()) {
    lcdDrawText(x, y, status.protocolSubName, flags);
  }
  else if (subType <= pdef->maxSubtype && pdef->subTypeString != nullptr) {
    lcdDrawTextAtIndex(x, y, pdef->subTypeString, subType, flags);
  }
  else {
    lcdDrawNumber(x, y, subType, flags);
  }
}
#endif

#if defined(SIMU)
BitmapBuffer _lcd(BMP_RGB565, LCD_W, LCD_H, displayBuf);
BitmapBuffer * lcd = &_lcd;

void DMAFillRect(uint16_t * dest, uint16_t destw, uint16_t desth, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
#if defined(PCBX10) && !defined(SIMU)
  x = destw - (x + w);
  y = desth - (y + h);
#endif

  for (int i=0; i<h; i++) {
    for (int j=0; j<w; j++) {
      dest[(y+i)*destw+x+j] = color;
    }
  }
}

void DMACopyBitmap(uint16_t * dest, uint16_t destw, uint16_t desth, uint16_t x, uint16_t y, const uint16_t * src, uint16_t srcw, uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w, uint16_t h)
{
#if defined(PCBX10) && !defined(SIMU)
  x = destw - (x + w);
  y = desth - (y + h);
  srcx = srcw - (srcx + w);
  srcy = srch - (srcy + h);
#endif

  for (int i=0; i<h; i++) {
    memcpy(dest+(y+i)*destw+x, src+(srcy+i)*srcw+srcx, 2*w);
  }
}

void DMACopyAlphaBitmap(uint16_t * dest, uint16_t destw, uint16_t desth, uint16_t x, uint16_t y, const uint16_t * src, uint16_t srcw, uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w, uint16_t h)
{
#if defined(PCBX10) && !defined(SIMU)
  x = destw - (x + w);
  y = desth - (y + h);
  srcx = srcw - (srcx + w);
  srcy = srch - (srcy + h);
#endif

  for (coord_t line=0; line<h; line++) {
    uint16_t * p = dest + (y+line)*destw + x;
    const uint16_t * q = src + (srcy+line)*srcw + srcx;
    for (coord_t col=0; col<w; col++) {
      uint8_t alpha = *q >> 12;
      uint8_t red = ((((*q >> 8) & 0x0f) << 1) * alpha + (*p >> 11) * (0x0f-alpha)) / 0x0f;
      uint8_t green = ((((*q >> 4) & 0x0f) << 2) * alpha + ((*p >> 5) & 0x3f) * (0x0f-alpha)) / 0x0f;
      uint8_t blue = ((((*q >> 0) & 0x0f) << 1) * alpha + ((*p >> 0) & 0x1f) * (0x0f-alpha)) / 0x0f;
      *p = (red << 11) + (green << 5) + (blue << 0);
      p++; q++;
    }
  }
}

void DMABitmapConvert(uint16_t * dest, const uint8_t * src, uint16_t w, uint16_t h, uint32_t format)
{
  if (format == DMA2D_ARGB4444) {
    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        *dest = ARGB(src[0], src[1], src[2], src[3]);
        ++dest;
        src += 4;
      }
    }
  }
  else {
    for (int row = 0; row < h; ++row) {
      for(int col = 0; col < w; ++col) {
        *dest = RGB(src[1], src[2], src[3]);
        ++dest;
        src += 4;
      }
    }
  }
}
#endif

//OW
//THANKS to Adafruit and its GFX library !
//https://learn.adafruit.com/adafruit-gfx-graphics-library

void lcdDrawCircleQuarter(coord_t x0, coord_t y0, int16_t r, uint8_t corners, LcdFlags att)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  if (corners >= 15) {
    lcdDrawPoint(x0, y0 + r, att);
    lcdDrawPoint(x0, y0 - r, att);
    lcdDrawPoint(x0 + r, y0, att);
    lcdDrawPoint(x0 - r, y0, att);
  }

  while (x < y) {
	if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
	}
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (corners & 0x04) {
      lcdDrawPoint(x0 + x, y0 + y, att);
      lcdDrawPoint(x0 + y, y0 + x, att);
    }
    if (corners & 0x02) {
      lcdDrawPoint(x0 + x, y0 - y, att);
      lcdDrawPoint(x0 + y, y0 - x, att);
    }
    if (corners & 0x08) {
      lcdDrawPoint(x0 - y, y0 + x, att);
      lcdDrawPoint(x0 - x, y0 + y, att);
    }
    if (corners & 0x01) {
      lcdDrawPoint(x0 - y, y0 - x, att);
      lcdDrawPoint(x0 - x, y0 - y, att);
    }
  }
}

void lcdFillCircleQuarter(coord_t x0, coord_t y0, int16_t r, uint8_t corners, LcdFlags att)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  if (corners >= 3) {
    lcd->drawSolidVerticalLine(x0, y0 - r, 2 * r +1, att);
  }
  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (x < (y + 1)) {
      if (corners & 0x01) {
    	lcd->drawSolidVerticalLine(x0 + x, y0 - y, 2 * y + 1, att);
      }
      if (corners & 0x02) {
    	lcd->drawSolidVerticalLine(x0 - x, y0 - y, 2 * y + 1, att);
      }
    }
    if (y != py) {
      if (corners & 0x01) {
    	lcd->drawSolidVerticalLine(x0 + py, y0 - px, 2 * px + 1, att);
      }
      if (corners & 0x02) {
    	lcd->drawSolidVerticalLine(x0 - py, y0 - px, 2 * px + 1, att);
      }
      py = y;
    }
    px = x;
  }
}

void lcdFillTriangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t x2, coord_t y2, LcdFlags flags)
{
  int32_t a, b;

  if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }
  if (y1 > y2) { SWAP(y2, y1); SWAP(x2, x1); }
  if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }

  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)
      a = x1;
    else if (x1 > b)
      b = x1;
    if (x2 < a)
      a = x2;
    else if (x2 > b)
      b = x2;
    lcd->drawSolidHorizontalLine(a, y0, b - a + 1, flags);
    return;
  }

  int32_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
  int32_t sa = 0, sb = 0;
  int32_t last = (y1 == y2) ? y1 : y1 - 1;
  int32_t y;

  for (y = y0; y <= last; y++) {
    a = x0 + sa / dy01;
    b = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    if (a > b) SWAP(a, b);
    lcd->drawSolidHorizontalLine(a, y, b - a + 1, flags);
  }

  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);

  for (; y <= y2; y++) {
    a = x1 + sa / dy12;
    b = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    if (a > b) SWAP(a, b);
    lcd->drawSolidHorizontalLine(a, y, b - a + 1, flags);
  }
}

// Cohen�Sutherland, see wikipedia

// Compute the bit code for a point (x, y) using the clip rectangle
// bounded diagonally by (xmin, ymin), and (xmax, ymax)
uint8_t ComputeOutCode(float x, float y, float xmin, float xmax, float ymin, float ymax)
{
  uint8_t code = 0;
  if (x < xmin)
    code |= 1;
  else if (x > xmax)
    code |= 2;
  if (y < ymin)
    code |= 8;
  else if (y > ymax)
    code |= 4;
  return code;
}

// clipping algorithm clips a line from P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with
// diagonal from (xmin, ymin) to (xmax, ymax).
void CohenSutherlandLineClipAndDraw(float x0, float y0, float x1, float y1, float xmin, float xmax, float ymin, float ymax, uint8_t pat, LcdFlags flags)
{
  uint8_t outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
  uint8_t outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
  bool accept = false;

  while (true) {
    if (!(outcode0 | outcode1)) {
      accept = true;
      break;
    }
    else if (outcode0 & outcode1) {
      break;
    }
    else {
      float x = 0.0f, y = 0.0f;
      uint8_t outcode = (outcode1 > outcode0) ? outcode1 : outcode0;
      if (outcode & 4) {
        x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
        y = ymax;
      }
      else if (outcode & 8) {
        x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
        y = ymin;
      }
      else if (outcode & 2) {
        y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
        x = xmax;
      }
      else if (outcode & 1) {
        y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
        x = xmin;
      }
      if (outcode == outcode0) {
        x0 = x;
        y0 = y;
        outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
      }
      else {
        x1 = x;
        y1 = y;
        outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
      }
    }
  }
  if (accept) {
    lcdDrawLine(x0+0.5f, y0+0.5f, x1+0.5f, y1+0.5f, pat, flags);
  }
}

void lcdDrawLineWithClipping(coord_t x0, coord_t y0, coord_t x1, coord_t y1, coord_t xmin, coord_t xmax, coord_t ymin, coord_t ymax, uint8_t pat, LcdFlags flags)
{
  CohenSutherlandLineClipAndDraw(x0, y0, x1, y1, xmin, xmax, ymin, ymax, pat, flags);
}

void lcdDrawHudRectangle(float pitch, float roll, coord_t xmin, coord_t xmax, coord_t ymin, coord_t ymax, LcdFlags flags)
{
  constexpr float GRADTORAD = 0.017453293f;

  float dx = sinf(GRADTORAD*roll) * pitch;
  float dy = cosf(GRADTORAD*roll) * pitch * 1.85f;
  float angle = tanf(-GRADTORAD*roll);
  float ox = 0.5f * (xmin + xmax) + dx;
  float oy = 0.5f * (ymin + ymax) + dy;
  int32_t ywidth = (ymax - ymin);

  if (roll == 0.0f) { // prevent divide by zero
	lcdDrawFilledRect(
      xmin, max(ymin, ymin + (coord_t)(ywidth/2 + (int32_t)dy)),
      xmax - xmin, min(ywidth, ywidth/2 - (int32_t)dy + (dy != 0.0f ? 1 : 0)), SOLID, flags);
  }
  else if (fabs(roll) >= 180.0f) {
    lcdDrawFilledRect(xmin, ymin, xmax - xmin, min(ywidth, ywidth/2 + (int32_t)fabsf(dy)), SOLID, flags);
  }
  else {
    bool inverted = (fabsf(roll) > 90.0f);
    bool fillNeeded = false;
    int32_t ybot = (inverted) ? 0 : LCD_H;

    if (roll > 0.0f) {
      for (int32_t s = 0; s < ywidth; s++) {
        int32_t yy = ymin + s;
        int32_t xx = ox + ((float)yy - oy) / angle; // + 0.5f; rounding not needed
        if (xx >= xmin && xx <= xmax) {
       	  lcd->drawSolidHorizontalLine(xx, yy, xmax - xx + 1, flags);
        }
        else if (xx < xmin) {
          ybot = (inverted) ? max(yy, ybot) + 1 : min(yy, ybot);
          fillNeeded = true;
        }
      }
    }
    else {
      for (int32_t s = 0; s < ywidth; s++) {
        int32_t yy = ymin + s;
        int32_t xx = ox + ((float)yy - oy) / angle; // + 0.5f; rounding not needed
        if (xx >= xmin && xx <= xmax) {
          lcd->drawSolidHorizontalLine(xmin, yy, xx - xmin, flags);
        }
        else if (xx > xmax) {
          ybot = (inverted) ? max(yy, ybot) + 1 : min(yy, ybot);
          fillNeeded = true;
        }
      }
    }

    if (fillNeeded) {
      int32_t ytop = (inverted) ? ymin : ybot;
      int32_t height = (inverted) ? ybot - ymin : ymax - ybot;
      lcdDrawFilledRect(xmin, ytop, xmax - xmin, height, SOLID, flags);
    }
  }
}
//OWEND
