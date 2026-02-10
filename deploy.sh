#!/bin/bash

# Vercel Deployment Script
# This script helps deploy all three projects to Vercel

echo "🚀 Physical AI Textbook - Vercel Deployment Helper"
echo "=================================================="
echo ""

# Check if Vercel CLI is installed
if ! command -v vercel &> /dev/null; then
    echo "❌ Vercel CLI not found!"
    echo "📦 Install it with: npm install -g vercel"
    exit 1
fi

echo "✅ Vercel CLI found"
echo ""

# Function to deploy a project
deploy_project() {
    local project_name=$1
    local project_dir=$2

    echo "📦 Deploying $project_name..."
    echo "Directory: $project_dir"

    if [ -d "$project_dir" ]; then
        cd "$project_dir" || exit
        vercel --prod
        cd - > /dev/null || exit
        echo "✅ $project_name deployed!"
    else
        echo "❌ Directory $project_dir not found!"
    fi
    echo ""
}

# Main deployment menu
echo "Select deployment option:"
echo "1. Deploy Main Docusaurus Site"
echo "2. Deploy Backend API"
echo "3. Deploy Frontend"
echo "4. Deploy All (Sequential)"
echo "5. Exit"
echo ""
read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        deploy_project "Main Docusaurus Site" "."
        ;;
    2)
        deploy_project "Backend API" "./backend"
        ;;
    3)
        deploy_project "Frontend" "./frontend"
        ;;
    4)
        echo "🚀 Deploying all projects..."
        echo ""
        deploy_project "Main Docusaurus Site" "."
        deploy_project "Backend API" "./backend"
        deploy_project "Frontend" "./frontend"
        echo "✅ All projects deployed!"
        ;;
    5)
        echo "👋 Exiting..."
        exit 0
        ;;
    *)
        echo "❌ Invalid choice!"
        exit 1
        ;;
esac

echo ""
echo "🎉 Deployment complete!"
echo "📊 Check your deployments at: https://vercel.com/dashboard"
