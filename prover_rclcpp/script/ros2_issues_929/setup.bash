# copied from ament_package/template/prefix_level/setup.bash

AMENT_SHELL=bash

# source setup.sh from same directory as this file
AMENT_CURRENT_PREFIX=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
# trace output
if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then
  echo ". \"$AMENT_CURRENT_PREFIX/setup.sh\""
fi

export ENV_CACHE_FILE="$HOME/.ros/env.cache"
export ENV_CACHE_LOCK_FILE="$HOME/.ros/.env.cache.lock"

mkdir -p $(dirname "$ENV_CACHE_FILE")

# If .env.cache.lock is locked, env.cache is being made. So cannot use it.
if $(flock -n -x "$ENV_CACHE_LOCK_FILE" true) && [ -f "${ENV_CACHE_FILE}" ]; then
  _cached_ordered_commands=$(cat "${ENV_CACHE_FILE}")
  eval "$_cached_ordered_commands"
  unset _ament_prefix_sh_source_script
  echo "Use cached environmnet !"
  echo "If you want to update cached environment, please remove \"${ENV_CACHE_FILE}\" !"
  echo "And re-execute \". setup.bash\" again."
else
  . "$AMENT_CURRENT_PREFIX/setup.sh"
fi

unset ENV_CACHE_FILE
unset ENV_CACHE_LOCK_FILE
