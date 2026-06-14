import fs from 'node:fs/promises';
import path from 'node:path';

const root = path.resolve(process.cwd(), 'site');
const htmlFiles = [];

async function walk(dir) {
  for (const entry of await fs.readdir(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      await walk(full);
    } else if (entry.isFile() && entry.name.endsWith('.html')) {
      htmlFiles.push(full);
    }
  }
}

function isExternal(url) {
  return /^(https?:|mailto:|javascript:|#)/i.test(url);
}

function normalizeTarget(sourceFile, rawUrl) {
  const clean = rawUrl.split('#')[0].split('?')[0];
  if (!clean) return null;
  return path.resolve(path.dirname(sourceFile), clean);
}

function collectLinks(content) {
  const links = [];
  const attrRe = /(?<!:)(href|src)="([^"]+)"/g;
  let match;
  while ((match = attrRe.exec(content))) {
    links.push(match[2]);
  }
  return links;
}

await walk(root);

const issues = [];
for (const file of htmlFiles) {
  const content = await fs.readFile(file, 'utf8');
  if (/\.md(\b|[#?])|href="[^"]*\.md|src="[^"]*\.md/i.test(content)) {
    issues.push({ file, type: 'markdown-reference', link: '.md reference still present' });
  }
  for (const link of collectLinks(content)) {
    if (isExternal(link)) continue;
    const target = normalizeTarget(file, link);
    if (!target) continue;
    try {
      await fs.access(target);
    } catch {
      issues.push({ file, type: 'missing-target', link, target });
    }
  }
}

for (const required of ['site/.nojekyll', 'site/404.html', 'site/index.html', 'site/app.js', 'site/theme.css', 'site/vendor/vue.global.prod.js']) {
  const full = path.resolve(process.cwd(), required);
  try {
    await fs.access(full);
  } catch {
    issues.push({ file: full, type: 'missing-required-file', link: required, target: full });
  }
}

if (issues.length) {
  for (const issue of issues) {
    console.error(`[${issue.type}] ${issue.file} -> ${issue.link}${issue.target ? ` (${issue.target})` : ''}`);
  }
  process.exitCode = 1;
} else {
  console.log(`OK: validated ${htmlFiles.length} HTML files under site/`);
}
