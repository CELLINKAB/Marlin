#!/usr/bin/python3

import argparse
import pathlib

def link_files():
    parser = argparse.ArgumentParser(description="Sets up Configuration.h and Configuration_adv.h for given device")
    parser.add_argument("device", choices=('Foton', 'Exocyte', 'Generic'))
    args = parser.parse_args()
    marlin_path = pathlib.Path("Marlin")
    config_path = pathlib.Path("config")

    if not all((marlin_path.is_dir(), config_path.is_dir())):
        print("Please run from Project root")
        return
    device_config = config_path / f"{args.device}_Configuration.h"
    device_config_adv = config_path / f"{args.device}_Configuration_adv.h"
    if not all((device_config.exists(), device_config_adv.exists())):
        print("Device configuration not found :(")
        return
    config_h = marlin_path / "Configuration.h"
    config_adv_h = marlin_path / "Configuration_adv.h"
    config_h.unlink(True)
    config_adv_h.unlink(True)
    config_h.symlink_to(device_config.resolve())
    config_adv_h.symlink_to(device_config_adv.resolve())

if __name__ == "__main__":
    link_files()