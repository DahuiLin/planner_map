.PHONY: help build up down logs restart clean

help: ## Mostrar esta ayuda
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

build: ## Construir los contenedores Docker
	docker-compose build

up: ## Iniciar todos los servicios
	docker-compose up -d

down: ## Detener todos los servicios
	docker-compose down

logs: ## Ver logs de todos los servicios
	docker-compose logs -f

logs-ros: ## Ver logs del servicio ROS2
	docker-compose logs -f ros2

logs-web: ## Ver logs del servicio web
	docker-compose logs -f web

restart: ## Reiniciar todos los servicios
	docker-compose restart

clean: ## Limpiar contenedores, redes y volúmenes
	docker-compose down -v
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

status: ## Ver estado de los servicios
	docker-compose ps

shell-ros: ## Abrir shell en contenedor ROS2
	docker-compose exec ros2 bash

shell-web: ## Abrir shell en contenedor web
	docker-compose exec web bash

dev: ## Modo desarrollo - build y up con logs
	docker-compose up --build

rebuild: ## Reconstruir sin cache
	docker-compose build --no-cache
