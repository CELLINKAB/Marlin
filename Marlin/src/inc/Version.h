
  /**
   * Auto Generated file by scripts/version.py
   * 
   */
#pragma once
#include <string_view>
    
#ifndef SHORT_BUILD_VERSION
    //#define SHORT_MARLIN_VERSION_STRING "2.1.2"
    #define SHORT_CELLINK_VERSION_STRING "2.1.0-marlinversion2.1"
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

#define VER_COMMIT_DATE "2023-05-08"
#define VER_FULL_BUILD_META_DATA "16841.Branch.issues-marlinversion2.Sha.fc6934dcbbb154418731fbb08a761f00b11f6d74" 
#define VER_SEM_VER "2.1.0-marlinversion2.1" 
#define VER_MAJOR 2
#define VER_MINOR 1
#define VER_PATCH 0
#define VER_BUILD_VERSION "exomarlin/v0.0.1-18-gfc6934dcbb" 
#define VER_BRANCH "issues/marlinversion2" 
#define VER_CURRENT_COMMIT "fc6934dcbbb154418731fbb08a761f00b11f6d74" 
                
#define VER_TIMESTAMP "2023-05-09 10.21"
#define VER_USER "ed" 
#define VER_BUILDTAG "local" 
        