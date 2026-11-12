#!/usr/bin/env bash
# Every ordered pair of the seven primitives for each seed, two at a time:
#   examples/native_boolean_fuzz.sh <binary> <log> <mode> <seed>...
set -u
binary=$1; log=$2; mode=$3; shift 3
for seed in "$@"; do for a in 0 1 2 3 4 5 6; do for b in 0 1 2 3 4 5 6; do echo "$a $b $seed $mode"; done; done; done |
xargs -P 2 -L 1 bash -c 'out=$(timeout 240 "$1" "$2" "$3" "$4" "$5" 2>/dev/null); code=$?
    if [ $code -ge 124 ]; then echo "FAIL $2 $3 $4 $5 all : timeout or crash, exit $code"; else echo "$out"; fi' fuzz "$binary" >> "$log"
