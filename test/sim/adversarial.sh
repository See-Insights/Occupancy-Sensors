#!/bin/bash
# Compatibility entry point. Regression checks now live in the main suite and
# exit nonzero for unexpected defects, rather than succeeding when a bug exists.
exec bash "$(dirname "$0")/run.sh" "$@"
