#!/bin/bash

# If it already exists, just update
if [ -d rdk-minimal/ ]
then
	cd rdk-minimal
	git pull origin main
else
	mkdir rdk-minimal
	cd rdk-minimal
	git init -b main
	git remote add origin git@github.com:viamrobotics/rdk.git
	git config pull.ff only
	git fetch --depth=1 origin main
	git sparse-checkout set --cone "proto" "buf.yaml" "buf.lock"
	git pull origin main
fi
