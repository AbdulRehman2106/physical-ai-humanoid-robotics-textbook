#!/usr/bin/env node

/**
 * Readability Scoring Tool for Physical AI Textbook
 * Checks Flesch-Kincaid grade level for all MDX files
 * Target: Grade 10-12 (constitution requirement)
 */

const fs = require('fs');
const path = require('path');
const textstat = require('textstat');

// Configuration
const TARGET_MIN_GRADE = 10;
const TARGET_MAX_GRADE = 12;
const DOCS_DIR = path.join(__dirname, '..', 'docs');

// Extract text content from MDX (remove JSX, code blocks, frontmatter)
function extractTextFromMDX(content) {
  // Remove frontmatter
  content = content.replace(/^---[\s\S]*?---/, '');

  // Remove JSX components
  content = content.replace(/<[^>]+>/g, '');

  // Remove code blocks
  content = content.replace(/```[\s\S]*?```/g, '');
  content = content.replace(/`[^`]+`/g, '');

  // Remove markdown links but keep text
  content = content.replace(/\[([^\]]+)\]\([^\)]+\)/g, '$1');

  // Remove markdown formatting
  content = content.replace(/[*_#]/g, '');

  return content.trim();
}

// Analyze a single file
function analyzeFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf-8');
  const text = extractTextFromMDX(content);

  if (text.length < 100) {
    return null; // Skip very short files
  }

  const grade = textstat.fleschKincaidGrade(text);
  const readingEase = textstat.fleschReadingEase(text);

  return {
    file: path.relative(DOCS_DIR, filePath),
    grade: parseFloat(grade.toFixed(1)),
    readingEase: parseFloat(readingEase.toFixed(1)),
    wordCount: text.split(/\s+/).length,
    status: grade >= TARGET_MIN_GRADE && grade <= TARGET_MAX_GRADE ? 'PASS' : 'FAIL',
  };
}

// Find all MDX files
function findMDXFiles(dir) {
  const files = [];

  function traverse(currentDir) {
    const items = fs.readdirSync(currentDir);

    for (const item of items) {
      const fullPath = path.join(currentDir, item);
      const stat = fs.statSync(fullPath);

      if (stat.isDirectory() && !item.startsWith('_')) {
        traverse(fullPath);
      } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
        files.push(fullPath);
      }
    }
  }

  traverse(dir);
  return files;
}

// Main execution
function main() {
  console.log('📊 Readability Analysis for Physical AI Textbook\n');
  console.log(`Target: Flesch-Kincaid Grade ${TARGET_MIN_GRADE}-${TARGET_MAX_GRADE}\n`);

  const files = findMDXFiles(DOCS_DIR);
  const results = [];

  for (const file of files) {
    const result = analyzeFile(file);
    if (result) {
      results.push(result);
    }
  }

  if (results.length === 0) {
    console.log('⚠️  No content files found to analyze.\n');
    return;
  }

  // Sort by grade level
  results.sort((a, b) => b.grade - a.grade);

  // Display results
  console.log('File                                    | Grade | Ease  | Words | Status');
  console.log('---------------------------------------------------------------------');

  for (const result of results) {
    const statusIcon = result.status === 'PASS' ? '✓' : '✗';
    const fileName = result.file.padEnd(40);
    const grade = result.grade.toFixed(1).padStart(5);
    const ease = result.readingEase.toFixed(1).padStart(5);
    const words = result.wordCount.toString().padStart(5);

    console.log(`${fileName} | ${grade} | ${ease} | ${words} | ${statusIcon} ${result.status}`);
  }

  // Summary
  const passed = results.filter(r => r.status === 'PASS').length;
  const failed = results.filter(r => r.status === 'FAIL').length;
  const avgGrade = results.reduce((sum, r) => sum + r.grade, 0) / results.length;

  console.log('\n📈 Summary:');
  console.log(`   Total files: ${results.length}`);
  console.log(`   Passed: ${passed}`);
  console.log(`   Failed: ${failed}`);
  console.log(`   Average grade: ${avgGrade.toFixed(1)}`);

  if (failed > 0) {
    console.log('\n⚠️  Some files are outside the target readability range.');
    process.exit(1);
  } else {
    console.log('\n✅ All files meet readability requirements!');
  }
}

main();
