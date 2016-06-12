#!/bin/bash

set -o errexit -o nounset

if [ "$TRAVIS_BRANCH" != "master" ]
then
  echo "This commit was made against the $TRAVIS_BRANCH and not the master! No deploy!"
  exit 0
fi

rev=$(git rev-parse --short HEAD)

cd ~/catkin_ws/build/tonav_doc

git init
git config user.name "Travis-ci"
git config user.email "nobody@nobody.com"

git remote add upstream "https://$GH_TOKEN@github.com/tomas789/tonav.git"
git fetch upstream
git reset upstream/gh-pages

echo "tonav-cname" > CNAME

touch .

git add -A .
git commit -m "rebuild pages at ${rev}"
git push -q upstream HEAD:gh-pages
