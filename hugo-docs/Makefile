# Makefile for Hugo Documentation Site

# Source and destination directories
DOCS_SRC := ../docs
HUGO_CONTENT := content/v1
THEME_DIR := themes/hugo-book
THEME_REPO := https://github.com/alex-shpak/hugo-book.git

# OS detection and shell setup for cross-platform compatibility
ifeq ($(OS),Windows_NT)
	SHELL := cmd.exe
	.SHELLFLAGS := /C
	THEME_CHECK = if not exist themes\hugo-book
	THEME_CLONE = git clone --depth 1 https://github.com/alex-shpak/hugo-book.git themes\hugo-book
	THEME_UPDATE = cd themes\hugo-book && git pull origin master
else
	SHELL := /bin/sh
	THEME_CHECK = [ ! -d themes/hugo-book ]
	THEME_CLONE = git clone --depth 1 https://github.com/alex-shpak/hugo-book.git themes/hugo-book
	THEME_UPDATE = cd themes/hugo-book && git pull origin master
endif

.PHONY: copy-docs
copy-docs:
	go run scripts/copy_docs.go
	@echo "All Markdown files copied from $(DOCS_SRC) to $(HUGO_CONTENT) using Go script"

.PHONY: theme
theme:
	@echo "Installing/updating hugo-book theme..."
ifeq ($(OS),Windows_NT)
	if not exist themes\hugo-book ( \
		git clone --depth 1 https://github.com/alex-shpak/hugo-book.git themes\hugo-book \
	) else ( \
		cd themes\hugo-book && git pull origin main \
	)
else
	if [ ! -d "themes/hugo-book" ]; then \
		git clone --depth 1 https://github.com/alex-shpak/hugo-book.git themes/hugo-book; \
	else \
		cd themes/hugo-book && git pull origin main; \
	fi
endif

.PHONY: update
update: theme copy-docs
	hugo

.PHONY: serve
serve: theme copy-docs
	hugo server --disableFastRender

.PHONY: clean
clean:
	rm -rf $(HUGO_CONTENT)/*

# GitHub Pages deployment
.PHONY: deploy
deploy: update
	@if [ -z "$$GITHUB_TOKEN" ]; then \
		echo "GITHUB_TOKEN environment variable is required for deployment."; \
		exit 1; \
	fi
	@echo "Deploying to GitHub Pages..."
	@hugo --minify --destination public
	@cd public && \
	 git init && \
	 git config user.name "github-actions[bot]" && \
	 git config user.email "github-actions[bot]@users.noreply.github.com" && \
	 git add . && \
	 git commit -m "Deploy to GitHub Pages" && \
	 git branch -M gh-pages && \
	 git remote add origin https://x-access-token:$$GITHUB_TOKEN@github.com/your/repo.git && \
	 git push --force origin gh-pages
	@echo "Deployment complete. Your site should be live on GitHub Pages."

# Cloudflare Pages deployment
.PHONY: cloudflare
cloudflare: update
	@echo "Preparing site for Cloudflare Pages deployment..."
	@hugo --minify --destination public
	@echo ""
	@echo "To deploy to Cloudflare Pages:"
	@echo "1. Push your repository (including the public/ folder) to your connected Git provider."
	@echo "2. In the Cloudflare Pages dashboard, set the build command to: make cloudflare"
	@echo "3. Set the output directory to: public"
	@echo "4. Trigger a deploy from Cloudflare Pages."
	@echo ""
	@echo "Alternatively, you can manually upload the contents of the public/ directory to your Cloudflare Pages project."
