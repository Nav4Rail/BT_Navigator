#!/usr/bin/env bash
set -eo pipefail

log() { echo "[$(date +'%Y-%m-%d %H:%M:%S')] [INFO] $*"; }
err() { echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ERROR] $*" >&2; }

AUTO_MODE=0
PROMPT=""
NO_RVIZ=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --auto) AUTO_MODE=1; shift ;;
    --no-rviz) NO_RVIZ=1; shift ;;
    *) PROMPT="$*"; break ;;
  esac
done

if [ -z "$PROMPT" ]; then
  read -rp "Description de la mission (prompt LLM) : " PROMPT
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BT_NAV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_MAIN_WS="${ROS_MAIN_WS:-$HOME/ros2_ws}"
MAP_FILE="${MAP_FILE:-$BT_NAV_DIR/maps/exploration_map.yaml}"
NAV2_PARAMS_SRC="${NAV2_PARAMS_SRC:-$BT_NAV_DIR/params/nav2_params.yaml}"
GENERATED_BT_XML="${GENERATED_BT_XML:-$BT_NAV_DIR/behavior_trees/__generated/turtlebot_mission.xml}"

TMP_PARAMS="/tmp/nav2_params_generated_bt.yaml"

cleanup() {
  if [ -n "${PIDS:-}" ]; then
    log "Arrêt des processes: $PIDS"
    # shellcheck disable=SC2086
    kill $PIDS >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

log "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

if [ -f "$ROS_MAIN_WS/install/setup.bash" ]; then
  log "Sourcing workspace principal: $ROS_MAIN_WS"
  # shellcheck disable=SC1091
  source "$ROS_MAIN_WS/install/setup.bash"
else
  log "Workspace principal non trouvé (install/setup.bash absent): $ROS_MAIN_WS"
fi

if [ ! -f "$NAV2_PARAMS_SRC" ]; then
  err "Params Nav2 introuvables: $NAV2_PARAMS_SRC"
  exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
  err "Map introuvable: $MAP_FILE"
  exit 1
fi

log "Génération du Behavior Tree via LLM → $GENERATED_BT_XML"
python3 "$SCRIPT_DIR/gen_turtlebot_mission_bt.py" --prompt "$PROMPT" --output "$GENERATED_BT_XML"

log "Aperçu du BT généré:"
echo "------------------------------------------------------------"
cat "$GENERATED_BT_XML"
echo
echo "------------------------------------------------------------"

if [ "$AUTO_MODE" -eq 0 ]; then
  while true; do
    read -r -p "Valider ce Behavior Tree pour Nav2 et lancer Gazebo/Nav2 ? [O/n] " REPLY
    ANSWER="${REPLY,,}"
    case "$ANSWER" in
      ""|"o"|"oui"|"y"|"yes") break ;;
      "n"|"non"|"no") log "Annulé."; exit 0 ;;
      *) echo "Réponse non reconnue. Merci de répondre par Oui ou Non." ;;
    esac
  done
else
  log "Mode automatique (--auto): validation ignorée."
fi

log "Création d'un params_file Nav2 temporaire avec default_nav_to_pose_bt_xml=$GENERATED_BT_XML"
cp "$NAV2_PARAMS_SRC" "$TMP_PARAMS"

# Remplace (ou ajoute si absent) le paramètre bt_navigator.default_nav_to_pose_bt_xml
if grep -qE "^[[:space:]]*default_nav_to_pose_bt_xml:" "$TMP_PARAMS"; then
  # note: | comme séparateur pour faciliter les chemins
  sed -i "s|^[[:space:]]*default_nav_to_pose_bt_xml:.*$|    default_nav_to_pose_bt_xml: $GENERATED_BT_XML|g" "$TMP_PARAMS"
else
  err "Paramètre default_nav_to_pose_bt_xml non trouvé dans $TMP_PARAMS (format inattendu)."
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
log "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

log "Lancement Gazebo (turtlebot3_world)..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
PIDS="$!"
sleep 2

log "Lancement Nav2 localization..."
ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:="$MAP_FILE" &
PIDS="$PIDS $!"
sleep 2

log "Lancement Nav2 navigation (params_file=$TMP_PARAMS)..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:="$TMP_PARAMS" &
PIDS="$PIDS $!"

if [ "$NO_RVIZ" -eq 0 ]; then
  log "Lancement RViz..."
  ros2 launch nav2_bringup rviz_launch.py
else
  log "--no-rviz: RViz non lancé. Attente (Ctrl+C pour quitter)."
  wait
fi


