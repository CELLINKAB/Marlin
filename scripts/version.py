#!/usr/bin/python3
import sys
sys.path.append('buildroot/share/PlatformIO/scripts')
import pioutil
from datetime import datetime
import json
import subprocess
import os
from os import path, pardir, getcwd,system
import sys
import gitversion

def get_current_commit():
    head_commit = subprocess.run("git rev-parse --verify HEAD", shell=True, stdout=subprocess.PIPE)
    return head_commit.stdout.rstrip().decode()

def get_current_branch():
    branch = subprocess.run("git symbolic-ref -q --short HEAD", shell=True, stdout=subprocess.PIPE)
    return branch.stdout.rstrip().decode()

def git_describe():
    describe = subprocess.run("git describe --tags --first-parent", shell=True, stdout=subprocess.PIPE)
    if describe.stdout.rstrip().decode() is None:
        describe = subprocess.run("git describe --tags --first-parent --always", shell=True, stdout=subprocess.PIPE)
    return describe.stdout.rstrip().decode()

def generate_header():
    return """
  /**
   * Auto Generated file by scripts/version.py
   * 
   */
#pragma once
#include <string_view>
    """
def generate_marlin(semver,majorminiopatch):
    return """
#ifndef SHORT_BUILD_VERSION
    //#define SHORT_MARLIN_VERSION_STRING "2.1.2"
    #define SHORT_CELLINK_VERSION_STRING "{}"
    #define SHORT_BUILD_VERSION "{}"
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
""".format(semver,majorminiopatch)

def process_gitversion():
    system("gitversion /output file")
    with open('GitVersion.json', encoding='utf-8-sig') as gitversionjson_file:
        gitversionjson = gitversionjson_file.read()
    return json.loads(gitversionjson)


def generate_gitversion(gv):
        gitdescribe = git_describe()
        gitbranch = get_current_branch()
        gitcommit = get_current_commit()

        return """
#define VER_COMMIT_DATE "{}"
#define VER_FULL_BUILD_META_DATA "{}" 
#define VER_SEM_VER "{}" 
#define VER_MAJOR {}
#define VER_MINOR {}
#define VER_PATCH {}
#define VER_BUILD_VERSION "{}" 
#define VER_BRANCH "{}" 
#define VER_CURRENT_COMMIT "{}" 
                """.format(gv["CommitDate"], gv["FullBuildMetaData"], gv["SemVer"],gv["Major"],gv["Minor"],gv["Patch"],gitdescribe, gitbranch, gitcommit)

def generate_env():
    timestamp = datetime.now().strftime("%Y-%m-%d %H.%M")
    user = os.environ["USER"]
    if globals().get('BUILD_TAG'):
        buildtag = os.environ["BUILD_TAG"]
    else:
        buildtag = "local"


    return """
#define VER_TIMESTAMP "{}"
#define VER_USER "{}" 
#define VER_BUILDTAG "{}" 
        """.format(timestamp, user, buildtag)

def make_versionfile():

    source_dir = path.join( 'Marlin','src', 'inc',)
    version_file = path.join(source_dir, 'Version.h')
    gv = process_gitversion()

    print("Current working dir: " + getcwd())
    print(generate_header())
    print(generate_marlin(gv["SemVer"],gv["MajorMinorPatch"]))
    print(generate_gitversion(gv))
    print(generate_env())

    with open(version_file, "w+") as f:
        f.write(generate_header())
        f.write(generate_marlin(gv["SemVer"],gv["MajorMinorPatch"]))
        f.write(generate_gitversion(gv))
        f.write(generate_env())

    print("Version file creation script success. Created:")



    with open(version_file, "r") as rf:
        contents = rf.read()
        print(contents)
def make_versionjson():
    timestamp = datetime.now().strftime("%Y-%m-%d %H.%M")
    user = os.environ["USER"]
    if globals().get('BUILD_TAG'):
        buildtag = os.environ["BUILD_TAG"]
    else:
        buildtag = "local"

    versionfiledict = {
        "user" : user,
        "buildtag" : buildtag,
        "timestamp" : timestamp
    }   
    json_object = json.dumps(versionfiledict, indent=4)   
    # Writing to sample.json
    with open("version.json", "w") as outfile:
        outfile.write(json_object)

if __name__ == "__main__":
    try:
        make_versionfile()
        make_versionjson()

    except Exception as e:
         print("Version file creation script failed. Details:", e)
else:
        #
    # From within PlatformIO use the loaded INI file
    #
    import pioutil
    if pioutil.is_pio_build():
        try:
            make_versionfile()
            make_versionjson()
        except Exception as e:
            print("Version file creation script failed. Details:", e)