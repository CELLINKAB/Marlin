<p align="center"><img src="buildroot/share/pixmaps/logo/marlin-outrun-nf-500.png" height="250" alt="MarlinFirmware's logo" /></p>

<h1 align="center">Marlin-Cell 3D Bio Printer Firmware</h1>

<p align="center">
    <a href="/LICENSE"><img alt="GPL-V3.0 License" src="https://img.shields.io/github/license/marlinfirmware/marlin.svg"></a>
    <a href="https://github.com/MarlinFirmware/Marlin/graphs/contributors"><img alt="Contributors" src="https://img.shields.io/github/contributors/marlinfirmware/marlin.svg"></a>
    <a href="https://github.com/MarlinFirmware/Marlin/releases"><img alt="Last Release Date" src="https://img.shields.io/github/release-date/MarlinFirmware/Marlin"></a>
    <a href="https://github.com/MarlinFirmware/Marlin/actions"><img alt="CI Status" src="https://github.com/MarlinFirmware/Marlin/actions/workflows/test-builds.yml/badge.svg"></a>
    <a href="https://github.com/sponsors/thinkyhead"><img alt="GitHub Sponsors" src="https://img.shields.io/github/sponsors/thinkyhead?color=db61a2"></a>
    <br />
    <a href="https://fosstodon.org/@marlinfirmware"><img alt="Follow MarlinFirmware on Mastodon" src="https://img.shields.io/mastodon/follow/109450200866020466?domain=https%3A%2F%2Ffosstodon.org&logoColor=%2300B&style=social"></a>
</p>

Additional documentation can be found at the [Marlin Home Page](https://marlinfw.org/).
Please test this firmware and let us know if it misbehaves in any way.


## Example Configurations



## Building Instructions

Unless you will be making changes to the configuration, or otherwise doing development in the repo, builds should be obtained from the [jenkins](https://jenkins.cellink.dev/view/BioCellX%20Current%20List/job/Common/job/Marlin%20Internal%20Build/).

To build and upload Cell-Marlin, use [Visual Studio Code](https://code.visualstudio.com/download) with the [PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) extension. You will also need a recent (>3.9) version of [Python](https://python.org).

Device configuration is selected with an included script `scripts/setup-config.py`, run the script without arguments to see current valid configurations. After picking a configuration, pick your target board from the evironment selector on the bottom bar.

#### Current Supported Configurations

<table>
<tr><td>Configuration</td><td>Board</td>
<tr><td>Exocyte</td><td>MYCORRHIZA_V1_1</td>
<tr><td>Foton</td><td>STM32H723Vx_btt</td>
<tr><td>Foton</td><td>STM32H743Vx_btt</td>
<tr><td>Generic</td><td>mega2560</td>
</table>

## Submitting Changes

All changes should be submitted as a Pull Request to the [dev](https://github.com/CELLINKAB/Marlin-Internal/tree/dev) branch. Changes which need to be applied to a release should still be submitted to dev first, and the PR should be tagged with "add to release". Changes should always be on the most recent commit, updated via rebase if needed. Please follow the [Upstream Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) to ease maintenance. 

Changes need to be reviewed by at least one person to be merged. Depending on which devices are affected by a change, more reviewers representing each affected device should be requested - or request to the github group for that project.

#### Release Patch Process
Changes merged with the "add to release" tag should have all of their changes cherry-picked to a patch branch. The patch branch should come from the release branch. Only these branches should be merged to active release branches, and all commits in a patch should be tested in dev first. Patch branches must be squashed before merging. Release patch merges should have multiple approvals from either the firmware github group, or the matching device github group.

# Marlin Resources

- [Marlin Documentation](https://marlinfw.org) - Official Marlin documentation
- [Marlin Discord](https://discord.gg/n5NJ59y) - Discuss issues with Marlin users and developers
- Facebook Group ["Marlin Firmware"](https://www.facebook.com/groups/1049718498464482/)
- RepRap.org [Marlin Forum](https://forums.reprap.org/list.php?415)
- Facebook Group ["Marlin Firmware for 3D Printers"](https://www.facebook.com/groups/3Dtechtalk/)
- [Marlin Configuration](https://www.youtube.com/results?search_query=marlin+configuration) on YouTube


## Upstream Maintaners

<table align="center">
<tr><td>Upstream Maintainer</td></tr>
<tr><td>
 🇺🇸  **Scott Lahteine**
       [@thinkyhead](https://github.com/thinkyhead)
       [<kbd>  Donate 💸  </kbd>](https://www.thinkyhead.com/donate-to-marlin)

</td><td>

 🇺🇸  **Roxanne Neufeld**
       [@Roxy-3D](https://github.com/Roxy-3D)

 🇺🇸  **Keith Bennett**
       [@thisiskeithb](https://github.com/thisiskeithb)
       [<kbd>  Donate 💸  </kbd>](https://github.com/sponsors/thisiskeithb)

 🇺🇸  **Jason Smith**
       [@sjasonsmith](https://github.com/sjasonsmith)

</td><td>

 🇧🇷  **Victor Oliveira**
       [@rhapsodyv](https://github.com/rhapsodyv)

 🇬🇧  **Chris Pepper**
       [@p3p](https://github.com/p3p)

🇳🇿  **Peter Ellens**
       [@ellensp](https://github.com/ellensp)
       [<kbd>  Donate 💸  </kbd>](https://ko-fi.com/ellensp)

</td><td>

 🇺🇸  **Bob Kuhn**
       [@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)

 🇳🇱  **Erik van der Zalm**
       [@ErikZalm](https://github.com/ErikZalm)
       [<kbd>  Donate 💸  </kbd>](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

</td></tr>
</table>

## License

Marlin is published under the [GPL license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
