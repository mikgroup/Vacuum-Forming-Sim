#!/usr/bin/env bash

echo -n "Enter your login for cs184.org: "
read CS184_LOGIN

ZIP_NAME=${CS184_LOGIN}"_proj4_clothsim.zip"

zip ${ZIP_NAME} \
  src/*.{cpp,h} \
  src/collision/*.{cpp,h}
zip -r ${ZIP_NAME} website

printf "\033[93mCreated a new file '${ZIP_NAME}' in your project directory.\n"
printf "You're not done yet! Go to \033[4mhttps://okpy.org/cal/cs184/sp17/clothsim/\033[24m in your browser to submit your project.\033[0m"
