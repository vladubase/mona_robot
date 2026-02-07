# MONA: Modular Open Navigating AMR

![CI Status](https://github.com/vladubase/mona_robot/actions/workflows/ci.yml/badge.svg)

**MONA** (Modular Open Navigating AMR) - это проект автономного мобильного робота для складской логистики. Система построена на базе ROS 2 Humble и использует контейнеризированную среду разработки для гарантии воспроизводимости кода.
## Основная цель проекта
Cоздание масштабируемой, безопасной и открытой архитектуры для управления флотом роботов, совместимой с индустриальными стандартами.
## Документация
Вся техническая документация находится в папке `docs/`:
1. **[Настройка окружения](docs/01_SETUP.md)** - Развертывание Docker-контейнера.
2. **[Руководство C++ разработчика](docs/02_CPP_GUIDE.md)** - Создание пакетов, нод и шаблоны кода.
3. **[Архитектура Lifecycle](docs/03_LIFECYCLE.md)** - Философия управляемых нод (Managed Nodes).
4. **[Рабочий процесс (Workflow)](docs/04_WORKFLOW.md)** - Сборка, очистка кэша и тесты.
5. **[Правила участия (Contributing)](docs/CONTRIBUTING.md)** - Git Flow, стандарты качества (ISO/VDA) и инструкции по Git.
## Быстрый старт
Проект запускается полностью в Docker. Установка ROS 2 на хост-машину не требуется.

```bash
# 1. Клонирование репозитория
git clone [https://github.com/vladubase/mona_robot.git](https://github.com/vladubase/mona_robot.git)
cd mona_robot

# 2. Запуск контейнера разработки
docker compose up -d

# 3. Вход в терминал контейнера
docker exec -it mona_dev bash
```