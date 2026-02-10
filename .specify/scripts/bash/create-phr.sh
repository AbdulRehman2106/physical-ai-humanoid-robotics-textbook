#!/bin/bash
# Simple PHR creation script
TITLE="$2"
STAGE="$4"
FEATURE="${6:-general}"

# Generate ID (simple increment)
ID=$(find history/prompts -name "*.prompt.md" 2>/dev/null | wc -l)
ID=$((ID + 1))

# Create slug
SLUG=$(echo "$TITLE" | tr '[:upper:]' '[:lower:]' | tr ' ' '-' | sed 's/[^a-z0-9-]//g')

# Determine path
if [ "$STAGE" = "constitution" ]; then
    DIR="history/prompts/constitution"
elif [ "$STAGE" = "general" ]; then
    DIR="history/prompts/general"
else
    DIR="history/prompts/$FEATURE"
fi

mkdir -p "$DIR"
FILEPATH="$DIR/${ID}-${SLUG}.${STAGE}.prompt.md"

echo "$FILEPATH"
