#!/bin/bash

# Script to update Hugging Face Spaces with latest backend code

echo "🚀 Updating Hugging Face Spaces Backend..."

# Check if huggingface-cli is installed
if ! command -v huggingface-cli &> /dev/null; then
    echo "❌ huggingface-cli not found. Installing..."
    pip install huggingface_hub
fi

# Login to Hugging Face (you'll need to provide your token)
echo "📝 Please login to Hugging Face:"
huggingface-cli login

# Clone your space
echo "📥 Cloning your Hugging Face Space..."
git clone https://huggingface.co/spaces/abdul18/rag-chatbot hf-space-temp

# Copy backend files
echo "📋 Copying updated backend files..."
cp -r backend/* hf-space-temp/

# Push to Hugging Face
cd hf-space-temp
git add .
git commit -m "Update CORS to include Vercel production URL"
git push

cd ..
rm -rf hf-space-temp

echo "✅ Hugging Face Spaces updated successfully!"
echo "🔄 Wait 1-2 minutes for the space to rebuild"
echo "🧪 Then test your chatbot at: https://physical-ai-textbook-wine.vercel.app"
