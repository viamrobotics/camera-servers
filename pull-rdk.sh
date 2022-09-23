#!/bin/bash

# If it already exists, just update
if [ -d api/ ]
then
	cd api
	git pull origin main
else
	mkdir api
	cd api
	git init -b main
	git remote add origin git@github.com:viamrobotics/api.git
	git config pull.ff only
	git fetch --depth=1 origin main
	git pull origin main
fi
