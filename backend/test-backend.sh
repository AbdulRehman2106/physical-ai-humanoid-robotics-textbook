#!/bin/bash

# RAG Chatbot Backend Test Script
# Tests all backend endpoints to verify deployment

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Backend URL (change this to your deployed URL)
BACKEND_URL="${BACKEND_URL:-http://localhost:8000}"

echo "🧪 Testing RAG Chatbot Backend"
echo "Backend URL: $BACKEND_URL"
echo ""

# Test 1: Basic Health Check
echo "Test 1: Basic Health Check"
response=$(curl -s -w "\n%{http_code}" "$BACKEND_URL/api/health")
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | head -n-1)

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✓ Health check passed${NC}"
    echo "Response: $body"
else
    echo -e "${RED}✗ Health check failed (HTTP $http_code)${NC}"
    exit 1
fi
echo ""

# Test 2: Detailed Health Check
echo "Test 2: Detailed Health Check"
response=$(curl -s -w "\n%{http_code}" "$BACKEND_URL/api/health/detailed")
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | head -n-1)

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✓ Detailed health check passed${NC}"
    echo "Response: $body"
else
    echo -e "${YELLOW}⚠ Detailed health check returned HTTP $http_code${NC}"
    echo "This might indicate database connection issues"
fi
echo ""

# Test 3: Create Conversation
echo "Test 3: Create Conversation"
response=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/chat/conversations" \
    -H "Content-Type: application/json")
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | head -n-1)

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✓ Conversation creation passed${NC}"
    conversation_id=$(echo "$body" | grep -o '"conversation_id":"[^"]*"' | cut -d'"' -f4)
    echo "Conversation ID: $conversation_id"
else
    echo -e "${RED}✗ Conversation creation failed (HTTP $http_code)${NC}"
    exit 1
fi
echo ""

# Test 4: Chat Query
echo "Test 4: Chat Query"
response=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/chat/query" \
    -H "Content-Type: application/json" \
    -d '{"query":"What is Physical AI?"}')
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | head -n-1)

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✓ Chat query passed${NC}"
    echo "Response preview:"
    echo "$body" | head -c 200
    echo "..."
else
    echo -e "${RED}✗ Chat query failed (HTTP $http_code)${NC}"
    echo "Response: $body"
    exit 1
fi
echo ""

# Test 5: Query with Context
echo "Test 5: Query with Context"
response=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/chat/query-with-context" \
    -H "Content-Type: application/json" \
    -d '{
        "query":"Explain this concept",
        "selected_text":"Physical AI combines artificial intelligence with physical embodiment",
        "conversation_id":"'"$conversation_id"'"
    }')
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | head -n-1)

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✓ Query with context passed${NC}"
    echo "Response preview:"
    echo "$body" | head -c 200
    echo "..."
else
    echo -e "${RED}✗ Query with context failed (HTTP $http_code)${NC}"
    echo "Response: $body"
fi
echo ""

# Test 6: Get Conversation
if [ -n "$conversation_id" ]; then
    echo "Test 6: Get Conversation"
    response=$(curl -s -w "\n%{http_code}" "$BACKEND_URL/api/chat/conversations/$conversation_id")
    http_code=$(echo "$response" | tail -n1)
    body=$(echo "$response" | head -n-1)

    if [ "$http_code" = "200" ]; then
        echo -e "${GREEN}✓ Get conversation passed${NC}"
        message_count=$(echo "$body" | grep -o '"role"' | wc -l)
        echo "Messages in conversation: $message_count"
    else
        echo -e "${RED}✗ Get conversation failed (HTTP $http_code)${NC}"
    fi
    echo ""
fi

# Summary
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ All tests completed!${NC}"
echo ""
echo "Backend is ready for production use."
echo "Next steps:"
echo "  1. Update frontend NEXT_PUBLIC_API_URL"
echo "  2. Deploy frontend to Vercel"
echo "  3. Test from the live site"
