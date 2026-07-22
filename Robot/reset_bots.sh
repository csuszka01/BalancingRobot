set -e

NAMESPACE="default"
LABEL_SELECTOR="app=robot"
PORT=8080

POD_IPS=$(kubectl get pods -n "$NAMESPACE" \
  -l "$LABEL_SELECTOR" \
  --field-selector status.phase=Running \
  -o jsonpath='{.items[*].status.podIP}')

if [ -z "$POD_IPS" ]; then
  echo "Error: No running pods found matching selector '${LABEL_SELECTOR}'!"
  exit 1
fi

read -r -a IP_ARRAY <<< "$POD_IPS"
echo "Found ${#IP_ARRAY[@]} target pod(s)."

for IP in "${IP_ARRAY[@]}"; do
  echo "Dispatching reset request to http://${IP}:${PORT}/reset..."
  
  # Fail silently if a single pod fails to respond, allowing others to finish
  curl -s -o /dev/null -w "%{http_code}" \
       -X POST "http://${IP}:${PORT}/reset" \
       --connect-timeout 2 \
       --max-time 3 &
done

wait

echo "Reset requests sent."