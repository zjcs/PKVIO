#! /bin/sh
git log --author="zjcs" --pretty=tformat: --numstat | awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "author: zjcs - added lines: %s, removed lines: %s, total lines: %s\n", add, subs, loc }' -
