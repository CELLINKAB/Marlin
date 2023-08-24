#/bin/bash
DEVICE=Exocyte
cp -f config/${DEVICE}_Configuration.h Marlin/Configuration.h
cp -f config/${DEVICE}_Configuration_adv.h Marlin/Configuration_adv.h