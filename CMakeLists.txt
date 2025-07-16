cmake_minimum_required(VERSION 3.0.0)

project(STTModBus VERSION 1.0 LANGUAGES CXX C)

add_executable(STTModBus src/main.cpp src/mqtt.cpp src/modbus.cpp src/mppt.cpp src/weatherst.cpp)

include_directories (includes)

# --- Tentar encontrar a biblioteca Mosquitto ---
# Opção 1: Usar find_package (Preferível)
find_package(Mosquitto CONFIG) # Tenta encontrar o arquivo de configuração MosquittoConfig.cmake ou mosquitto-config.cmake
# Se a linha acima falhar, remova-a e tente a Opção 2

if(Mosquitto_FOUND)
    message(STATUS "Biblioteca Mosquitto encontrada usando find_package.")
    # Inclui os diretórios de cabeçalho
    target_include_directories(STTModBus PUBLIC ${Mosquitto_INCLUDE_DIRS})
    # Vincula a biblioteca ao executável
    target_link_libraries(STTModBus PUBLIC ${Mosquitto_LIBRARIES})
else()
    message(WARNING "find_package(Mosquitto) falhou. Tentando com find_library...")

    # Opção 2: Usar find_library e find_path (Alternativa se find_package falhar)
    # 1. Encontrar o cabeçalho (mosquitto.h)
    find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h
        PATHS /usr/include /usr/local/include
        PATH_SUFFIXES mosquitto # Onde o mosquitto.h pode estar (ex: /usr/include/mosquitto/mosquitto.h)
        REQUIRED
    )

    # 2. Encontrar a biblioteca (.so ou .a)
    find_library(MOSQUITTO_LIB
        NAMES mosquitto libmosquitto # Tenta nomes comuns da biblioteca
        PATHS /usr/lib /usr/local/lib
        REQUIRED
    )

    if(MOSQUITTO_INCLUDE_DIR AND MOSQUITTO_LIB)
        message(STATUS "Biblioteca Mosquitto encontrada usando find_path/find_library.")
        # Inclui o diretório de cabeçalho
        target_include_directories(STTModBus PUBLIC ${MOSQUITTO_INCLUDE_DIR})
        # Vincula a biblioteca ao executável
        target_link_libraries(STTModBus PUBLIC ${MOSQUITTO_LIB})
    else()
        message(FATAL_ERROR "Não foi possível encontrar a biblioteca Mosquitto ou seus cabeçalhos.")
    endif()
endif()

find_package(gpiod CONFIG)

if(gpiod_FOUND)
    message(STATUS "Biblioteca gpiod encontrada usando find_package.")
    target_include_directories(STTModBus PUBLIC ${gpiod_INCLUDE_DIRS})
    target_link_libraries(STTModBus PUBLIC ${gpiod_LIBRARIES})
else()
    message(WARNING "find_package(gpiod) falhou. Tentando com find_library...")

    # Opção 2: Usar find_library e find_path (Alternativa se find_package falhar)
    # 1. Encontrar o cabeçalho (gpiod.h)
    find_path(GPIOD_INCLUDE_DIR gpiod.h
        PATHS /usr/include /usr/local/include
        PATH_SUFFIXES gpiod # Pode estar em /usr/include/gpiod/gpiod.h
        REQUIRED
    )

    # 2. Encontrar a biblioteca (.so ou .a)
    find_library(GPIOD_LIB
        NAMES gpiod libgpiod # Nomes comuns da biblioteca
        PATHS /usr/lib /usr/local/lib
        REQUIRED
    )

    if(GPIOD_INCLUDE_DIR AND GPIOD_LIB)
        message(STATUS "Biblioteca gpiod encontrada usando find_path/find_library.")
        target_include_directories(STTModBus PUBLIC ${GPIOD_INCLUDE_DIR})
        target_link_libraries(STTModBus PUBLIC ${GPIOD_LIB})
    else()
        message(FATAL_ERROR "Não foi possível encontrar a biblioteca gpiod ou seus cabeçalhos. Certifique-se de que libgpiod-dev está instalada.")
    endif()
endif()

set(CMAKE_SYSTEM_NAME Linux)

set_target_properties(STTModBus PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "bin"
    ARCHIVE_OUTPUT_DIRECTORY "lib"
    LIBRARY_OUTPUT_DIRECTORY "lib"
)
