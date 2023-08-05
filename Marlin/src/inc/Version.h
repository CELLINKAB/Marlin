
  /**
   * Auto Generated file by scripts/version.py
   * 
   */
#pragma once
#include <string_view>
    
#ifndef SHORT_BUILD_VERSION
    //#define SHORT_MARLIN_VERSION_STRING "2.1.2"
    #define SHORT_CELLINK_VERSION_STRING "2.0.9-issues-sdcardflash.1"
    #define SHORT_BUILD_VERSION "2.0.9"
#endif
#ifndef DETAILED_BUILD_VERSION
    #define DETAILED_BUILD_VERSION "Cellink-Marlin " SHORT_CELLINK_VERSION_STRING
    //# " (Marlin " SHORT_MARLIN_VERSION_STRING ")"
#endif
#ifndef STRING_DISTRIBUTION_DATE
    #define STRING_DISTRIBUTION_DATE "2023-02-14"
#endif
#define MARLIN_HEX_VERSION 02000903
#ifndef REQUIRED_CONFIGURATION_H_VERSION
  #define REQUIRED_CONFIGURATION_H_VERSION MARLIN_HEX_VERSION
#endif
#ifndef REQUIRED_CONFIGURATION_ADV_H_VERSION
  #define REQUIRED_CONFIGURATION_ADV_H_VERSION MARLIN_HEX_VERSION
#endif
/**
 * Define a generic printer name to be output to the LCD after booting Marlin.
 */
#ifndef MACHINE_NAMEmajorminiopatch
    #define MACHINE_NAME "Lumen X gen3"
#endif
#ifndef PROTOCOL_VERSION
  #define PROTOCOL_VERSION "1.0"
#endif
#ifndef SOURCE_CODE_URL
  #define SOURCE_CODE_URL "github.com/CELLINKAB/Marlin"
#endif
/**
 * Default generic printer UUID.
 */
#ifndef DEFAULT_MACHINE_UUID
  #define DEFAULT_MACHINE_UUID "cede2a2f-41a2-4748-9b12-c55c62f367ff"
#endif
  /**
   * The WEBSITE_URL is the location where users can get more information such as
   * documentation about a specific Marlin release. Displayed in the Info Menu.
   */
#ifndef WEBSITE_URL
  #define WEBSITE_URL "cellink.com"
#endif

#define VER_COMMIT_DATE "2023-06-09"
#define VER_FULL_BUILD_META_DATA "82.Branch.issues-sdcardflash.Sha.9c53a51fc8cc0afaa41ede01989727ca087e8dca" 
#define VER_SEM_VER "2.0.9-issues-sdcardflash.1" 
#define VER_MAJOR 2
#define VER_MINOR 0
#define VER_PATCH 9
#define VER_BUILD_VERSION "skr2_dev-25-g0f01e3f" 
#define VER_BRANCH "issue/offset-tweaks" 
#define VER_CURRENT_COMMIT "0f01e3f04c4c9f262fcd9bd370ef9f3028bc8ac5" 
                
#define VER_TIMESTAMP "2023-07-11 16.03"
#define VER_USER "nouser" 
#define VER_BUILDTAG "local" 
        