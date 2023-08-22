
  /**
   * Auto Generated file by scripts/version.py
   * 
   */
#pragma once
    
#ifndef SHORT_BUILD_VERSION
    #define SHORT_BUILD_VERSION "2.1.2.1"
    #define SHORT_CELLINK_VERSION_STRING SHORT_BUILD_VERSION "-issues-addbuildtage.1"
#endif
#ifndef DETAILED_BUILD_VERSION
    #define DETAILED_BUILD_VERSION "Cellink-Marlin " SHORT_CELLINK_VERSION_STRING
    //# " (Marlin " SHORT_MARLIN_VERSION_STRING ")"
#endif
#ifndef STRING_DISTRIBUTION_DATE
  #define STRING_DISTRIBUTION_DATE "0000-00-00"
#endif

/**
 * Minimum Configuration.h and Configuration_adv.h file versions.
 * Set based on the release version number. Used to catch an attempt to use
 * older configurations. Override these if using a custom versioning scheme
 * to alert users to major changes.
 */

#define MARLIN_HEX_VERSION 02010201
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

#define VER_COMMIT_DATE "YYYY-MM-DD"
#define VER_FULL_BUILD_META_DATA "gitversion.FullBuildMetaData" 
#define VER_SEM_VER "gitversion.SemVer" 
#define VER_MAJOR 0
#define VER_MINOR 0
#define VER_PATCH 0
#define VER_BUILD_VERSION "git_describe" 
#define VER_BRANCH "git HEAD" 
#define VER_CURRENT_COMMIT "ffffffffffffffffffffffffffffffffffffffff" 

#define VER_TIMESTAMP "YYYY-MM-DD HH.MM"
#define VER_USER "default" 
#define VER_BUILDTAG "default" 
