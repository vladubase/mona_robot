# ==============================================================================
# Copyright 2026 vladubase
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

COMPOSE ?= docker compose

# Host-specific values (user IDs, GPU vendor, /dev/dri group IDs and Mesa source)
# are detected on every invocation and exported to Docker Compose.
HOST_ENV := $(shell ./scripts/detect_host.bash)
$(foreach kv,$(HOST_ENV),$(eval export $(kv)))

.PHONY: build rebuild up up-gpu down logs ps shell format_code ci doctor help \
	b r u ug d l p s f c

help: ## Show available Makefile targets
	@echo "Available commands:"
	@grep -E '^[a-zA-Z0-9_-]+:.*##' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS=":.*## "}; {printf "  \033[1;32m%-15s\033[0m %s\n", $$1, $$2}'

build: ## Build docker images (uses cache, fast)
	$(COMPOSE) build

rebuild: ## Full clean build of docker images (no-cache) and restart
	$(COMPOSE) build --no-cache

up: ## Start dev container (CPU-only, builds if missing)
	$(COMPOSE) up -d --build

up-gpu: ## Start dev container with NVIDIA GPU
	$(COMPOSE) -f docker-compose.yml -f docker-compose.gpu.yml up -d --build

down: ## Stop containers
	$(COMPOSE) down

logs: ## Tail docker compose logs
	$(COMPOSE) logs -f

ps: ## List docker compose services
	$(COMPOSE) ps

shell: ## Attach to the dev container's bash shell
	$(COMPOSE) up -d dev
	$(COMPOSE) exec -it dev bash

format_code: ## Format C++/Python/CMake and verify them with the CI linters (dev container)
	$(COMPOSE) up -d dev
	$(COMPOSE) exec dev bash -c "./scripts/format_code.bash"

ci: ## Run CI checks inside the dev container
	$(COMPOSE) up -d dev
	$(COMPOSE) exec dev bash -c "./scripts/run_ci_checks.bash"

doctor: ## Show detected host values and verify GPU acceleration in the container
	@./scripts/detect_host.bash
	@$(COMPOSE) run --rm --no-deps dev bash -c '\
		glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|Accelerated"; \
		if [ "$(GPU_VENDOR)" != "none" ] && glxinfo -B 2>/dev/null | grep -q llvmpipe; then \
			echo "[FAIL] Software rendering (llvmpipe) although host GPU is $(GPU_VENDOR)"; \
			exit 1; \
		fi'

world: ## Start Gazebo world (run on host)
	./scripts/start_world.bash

robot: ## Start robot agent (run on host)
	./scripts/start_1_robot.bash

# Short aliases for fast TAB completion
b: ## alias: build
	$(MAKE) build

r: ## alias: rebuild
	$(MAKE) rebuild

u: ## alias: up
	$(MAKE) up

ug: ## alias: up-gpu
	$(MAKE) up-gpu

d: ## alias: down
	$(MAKE) down

l: ## alias: logs
	$(MAKE) logs

p: ## alias: ps
	$(MAKE) ps

s: ## alias: shell
	$(MAKE) shell

f: ## alias: format_code
	$(MAKE) format_code

c: ## alias: ci
	$(MAKE) ci

w: ## alias: world
	$(MAKE) world

ro: ## alias: robot
	$(MAKE) robot