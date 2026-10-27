# Разработка

Устройство игры описано в [docs/architecture.md](docs/architecture.md).

## Правила

1. Движок подключается только через `laiue::bootstrap` и таблицы сервисов
   модулей, загруженных по имени. Внутренние заголовки движка недоступны.
2. Каждая технология движка необязательна: код проверяет таблицу сервиса и
   его размер и продолжает работу без неё. Новый модуль получает случай в
   `foxity.client_modules.*`.
3. `src/sim` детерминирован: только стандартный C и целые числа, никакого
   времени ОС, случайности ОС, плавающей точки, потоков без фиксированного
   порядка и платформенных ветвлений. Маршрутизатор подключается снаружи и
   обязан быть детерминированным.
4. Кадр не ждёт: построение мешей идёт в бюджете времени на кадр, всё
   тяжёлое — заранее или по частям.
5. Повторно используемый функционал переносится в
   [laiue](https://github.com/landiunload/laiue) отдельным изменением с
   тестами и контрактом без игровых терминов.

## Проверки

```powershell
cmake --preset windows-msvc
cmake --build --preset windows-msvc-debug --parallel
ctest --preset windows-msvc-debug --no-tests=error --output-on-failure
cmake --build --preset windows-msvc-release --parallel
ctest --preset windows-msvc-release --no-tests=error --output-on-failure
cmake --preset windows-clang -B build/tidy -DFOX_ENABLE_CLANG_TIDY=ON
cmake --build build/tidy --config Debug --parallel
git diff --check
```

Ошибок и предупреждений быть не должно, подавлять их нельзя: причина
исправляется в коде. CI дополнительно собирает Linux (GCC, Clang и GCC с
AddressSanitizer, UndefinedBehaviorSanitizer и проверкой утечек) и macOS.

## Проверочная сборка

`-DFOX_REVIEW_BUILD=ON` собирает рядом с игрой `FoxityReview` — только для
проверяющих, в выпуск она не входит. В ней P сохраняет кадр как
`review/shot-NNN.bmp` (каталог меняет `--review-dir`) и открывает строку для
комментария: Enter сохраняет его рядом в `shot-NNN.txt` вместе с положением
камеры, Esc — без комментария. Пока строка открыта, клавиши игры молчат.
