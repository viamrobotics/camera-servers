#!/bin/bash

for i in "$@"; do
  case $i in
    --self-update)
      UPDATE=1
      echo "SMURF1"
      ;;
    --install-service)
      INSTALL_SERVICE=1
      echo "SMURF2"
      ;;
    *)
      # unknown option
      echo "SMURF3 $i"
      ;;
  esac
done

if [[ -v $UPDATE ]]
then
	echo "SMURF UPDATE: $APPIMAGE"
	$APPDIR/usr/bin/appimage-update "$APPIMAGE"
	exit
fi

if [[ -v $INSTALL_SERVICE ]]
then
	echo "Would install systemd services here"
	exit
fi

$MAIN_PROG $@