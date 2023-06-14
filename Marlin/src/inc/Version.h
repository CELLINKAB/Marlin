
  /**
   * Auto Generated file by scripts/version.py
   * 
   */
#pragma once
#include <string_view>
    
#ifndef SHORT_BUILD_VERSION
    //#define SHORT_MARLIN_VERSION_STRING "2.1.2"
    #define SHORT_CELLINK_VERSION_STRING "2.1.0-issues-addbuildtage.1"
    #define SHORT_BUILD_VERSION "2.1.0"
#endif
#ifndef DETAILED_BUILD_VERSION
    #define DETAILED_BUILD_VERSION "Cellink-Marlin " SHORT_CELLINK_VERSION_STRING
    //# " (Marlin " SHORT_MARLIN_VERSION_STRING ")"
#endif
#ifndef STRING_DISTRIBUTION_DATE
    #define STRING_DISTRIBUTION_DATE "2023-02-14"
#endif
#define MARLIN_HEX_VERSION 02010200
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
    #define MACHINE_NAME "Bio Cell X"
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

#define VER_COMMIT_DATE "2023-05-31"
#define VER_FULL_BUILD_META_DATA "16947.Branch.issues-addbuildtage.Sha.3e9898097f54e5908ca7a6494ea9c917f86d7e5a" 
#define VER_SEM_VER "2.1.0-issues-addbuildtage.1" 
#define VER_MAJOR 2
#define VER_MINOR 1
#define VER_PATCH 0
#define VER_BUILD_VERSION "exomarlin/v0.0.1-29-g3e9898097f" 
#define VER_BRANCH "issues/addbuildtage" 
#define VER_CURRENT_COMMIT "3e9898097f54e5908ca7a6494ea9c917f86d7e5a" 
                
#define VER_TIMESTAMP "2023-06-14 10.10"
#define VER_USER "ed" 
#define VER_BUILDTAG "me" 
        