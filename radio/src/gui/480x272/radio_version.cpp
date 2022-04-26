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

#include "opentx.h"
#include "options.h"

#if defined(PXX2) || defined(MULTIMODULE)
constexpr coord_t COLUMN2_X = 200;

void drawPXX2Version(coord_t x, coord_t y, PXX2Version version)
{
  if (version.major == 0xFF && version.minor == 0x0F && version.revision == 0x0F) {
    lcdDrawText(x, y, "---");
  }
  else {
    lcdDrawNumber(x, y, 1 + version.major, LEFT);
    lcdDrawChar(lcdNextPos, y, '.');
    lcdDrawNumber(lcdNextPos, y, version.minor, LEFT);
    lcdDrawChar(lcdNextPos, y, '.');
    lcdDrawNumber(lcdNextPos, y, version.revision, LEFT);
  }
}

void drawPXX2FullVersion(coord_t x, coord_t y, PXX2Version hwVersion, PXX2Version swVersion)
{
  drawPXX2Version(x, y, hwVersion);
  lcdDrawText(lcdNextPos, y, "/");
  drawPXX2Version(lcdNextPos, y, swVersion);
}

bool menuRadioModulesVersion(event_t event)
{
  if (menuEvent) {
    moduleState[INTERNAL_MODULE].mode = MODULE_MODE_NORMAL;
    moduleState[EXTERNAL_MODULE].mode = MODULE_MODE_NORMAL;
    return false;
  }

  drawMenuTemplate(STR_MENU_MODULES_RX_VERSION, 0, RADIO_ICONS, OPTION_MENU_TITLE_BAR);

  if (event == EVT_ENTRY) {
    memclear(&reusableBuffer.hardwareAndSettings.modules, sizeof(reusableBuffer.hardwareAndSettings.modules));
  }

  if (event == EVT_ENTRY || get_tmr10ms() >= reusableBuffer.hardwareAndSettings.updateTime) {
    if (isModulePXX2(INTERNAL_MODULE) && IS_INTERNAL_MODULE_ON()) {
      moduleState[INTERNAL_MODULE].readModuleInformation(&reusableBuffer.hardwareAndSettings.modules[INTERNAL_MODULE], PXX2_HW_INFO_TX_ID, PXX2_MAX_RECEIVERS_PER_MODULE - 1);
    }

    if (isModulePXX2(EXTERNAL_MODULE) && IS_EXTERNAL_MODULE_ON()) {
      moduleState[EXTERNAL_MODULE].readModuleInformation(&reusableBuffer.hardwareAndSettings.modules[EXTERNAL_MODULE], PXX2_HW_INFO_TX_ID, PXX2_MAX_RECEIVERS_PER_MODULE - 1);
    }

    reusableBuffer.hardwareAndSettings.updateTime = get_tmr10ms() + 1000 /* 10s*/;
  }

  coord_t y = MENU_BODY_TOP - menuVerticalOffset * FH;

  for (uint8_t module=0; module<NUM_MODULES; module++) {
    // Label
    if (y >= MENU_BODY_TOP && y < MENU_FOOTER_TOP) {
      if (module == INTERNAL_MODULE)
        lcdDrawText(MENUS_MARGIN_LEFT, y, STR_INTERNAL_MODULE);
      else if (module == EXTERNAL_MODULE)
        lcdDrawText(MENUS_MARGIN_LEFT, y, STR_EXTERNAL_MODULE);
    }
    y += FH;

    // Module model
    if (y >= MENU_BODY_TOP && y < MENU_FOOTER_TOP) {
      lcdDrawText(MENUS_MARGIN_LEFT + INDENT_WIDTH, y, STR_MODULE);
      if ((module == INTERNAL_MODULE && !IS_INTERNAL_MODULE_ON()) ||
          (module == EXTERNAL_MODULE && !IS_EXTERNAL_MODULE_ON())) {
        lcdDrawText(COLUMN2_X, y, STR_OFF);
        y += FH;
        continue;
      }
#if defined(MULTIMODULE)
      if (isModuleMultimodule(module)) {
        char statusText[64];
        lcdDrawText(COLUMN2_X, y, "Multimodule");
        y += FH;
        getMultiModuleStatus(INTERNAL_MODULE).getStatusString(statusText);
        lcdDrawText(COLUMN2_X, y, statusText);
        y += FH;
        continue;
      }
#endif
      if (!isModulePXX2(module)) {
        lcdDrawText(COLUMN2_X, y, STR_NO_INFORMATION);
        y += FH;
        continue;
      }
      uint8_t modelId = reusableBuffer.hardwareAndSettings.modules[module]
                            .information.modelID;
      lcdDrawText(COLUMN2_X, y, getPXX2ModuleName(modelId));
    }
    y += FH;

    // Module version
    if (y >= MENU_BODY_TOP && y < MENU_FOOTER_TOP) {
      if (reusableBuffer.hardwareAndSettings.modules[module].information.modelID) {
        drawPXX2FullVersion(COLUMN2_X, y, reusableBuffer.hardwareAndSettings.modules[module].information.hwVersion, reusableBuffer.hardwareAndSettings.modules[module].information.swVersion);
        static const char * variants[] = {"FCC", "EU", "FLEX"};
        uint8_t variant = reusableBuffer.hardwareAndSettings.modules[module].information.variant - 1;
        if (variant < DIM(variants)) {
          lcdDrawText(lcdNextPos + 1, y, variants[variant]);
        }
      }
    }
    y += FH;

    for (uint8_t receiver=0; receiver<PXX2_MAX_RECEIVERS_PER_MODULE; receiver++) {
      if (reusableBuffer.hardwareAndSettings.modules[module].receivers[receiver].information.modelID && reusableBuffer.hardwareAndSettings.modules[module].receivers[receiver].timestamp < get_tmr10ms() + 2000) {
        // Receiver model
        if (y >= MENU_BODY_TOP && y < MENU_FOOTER_TOP) {
          lcdDrawText(MENUS_MARGIN_LEFT + INDENT_WIDTH, y, STR_RECEIVER);
          lcdDrawNumber(lcdNextPos + 2, y, receiver + 1);
          uint8_t modelId = reusableBuffer.hardwareAndSettings.modules[module].receivers[receiver].information.modelID;
          lcdDrawText(COLUMN2_X, y, getPXX2ReceiverName(modelId));
        }
        y += FH;

        // Receiver version
        if (y >= MENU_BODY_TOP && y < MENU_FOOTER_TOP) {
          drawPXX2FullVersion(COLUMN2_X, y, reusableBuffer.hardwareAndSettings.modules[module].receivers[receiver].information.hwVersion, reusableBuffer.hardwareAndSettings.modules[module].receivers[receiver].information.swVersion);
        }
        y += FH;
      }
    }
  }

  uint8_t lines = (y - (FH + 1)) / FH + menuVerticalOffset;
  if (lines > NUM_BODY_LINES) {
    drawVerticalScrollbar(LCD_W-1, FH, LCD_H-FH, menuVerticalOffset, lines, NUM_BODY_LINES);
  }

  switch(event) {
    case EVT_ROTARY_LEFT:
      if (lines > NUM_BODY_LINES) {
        if (menuVerticalOffset-- == 0)
          menuVerticalOffset = lines - 1;
      }
      break;

    case EVT_ROTARY_RIGHT:
      if (lines > NUM_BODY_LINES) {
        if (++menuVerticalOffset + NUM_BODY_LINES > lines)
          menuVerticalOffset = 0;
      }
      break;

    case EVT_KEY_BREAK(KEY_EXIT):
      if (menuVerticalOffset != 0)
        menuVerticalOffset = 0;
      else
        popMenu();
      break;
  }

  return true;
}
#endif

bool menuRadioVersion(event_t event)
{
  char id[27];
  getCPUUniqueID(id);

  SIMPLE_MENU(STR_MENUVERSION, RADIO_ICONS, menuTabGeneral, MENU_RADIO_VERSION, 1);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP, fw_stamp);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP + FH, vers_stamp);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP + 2*FH, date_stamp);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP + 3*FH, time_stamp);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP + 4*FH, eeprom_stamp);
  lcdDrawText(MENUS_MARGIN_LEFT, MENU_CONTENT_TOP + 5*FH, "UID:");
  lcdDrawText(MENUS_MARGIN_LEFT + 64, MENU_CONTENT_TOP + 5*FH, id);

  coord_t y = MENU_CONTENT_TOP + 5*FH;
  lcdDrawText(MENUS_MARGIN_LEFT, y, "OPTS:");
  lcdNextPos = MENUS_MARGIN_LEFT + 64;

  for (uint8_t i=0; options[i]; i++) {
    const char * option = options[i];
    coord_t width = getTextWidth(option);

    if ((lcdNextPos + 5 + width) > LCD_W) {
      lcdDrawText(lcdNextPos, y, ",");
      lcdNextPos = MENUS_MARGIN_LEFT;
      y += FH;
    }
    if (i > 0 && lcdNextPos !=MENUS_MARGIN_LEFT) {
      lcdDrawText(lcdNextPos, y, ", ");
    }
    lcdDrawText(lcdNextPos, y, option);
  }

#if defined(PXX2) || defined(MULTIMODULE)
  y += FH + FH / 2;

  lcdDrawText(MENUS_MARGIN_LEFT, y, BUTTON(TR_MODULES_RX_VERSION), menuVerticalPosition == 0 ? INVERS : 0);
  y += FH;
  if (menuVerticalPosition == 0 && event == EVT_KEY_BREAK(KEY_ENTER)) {
    s_editMode = EDIT_SELECT_FIELD;
    pushMenu(menuRadioModulesVersion);
  }
#endif

  return true;
}
