#!/bin/bash
# This script just checks if all the endpoints work. No validation.

curl http://localhost:1095/logbook

curl --data '{"text": "hoi", "source": "magic"}' -X POST http://localhost:1095/logbook --header "Content-Type:application/json"

curl --data '{"timestamp": "123"}' -X PUT http://localhost:1095/logbook/1 --header "Content-Type:application/json"