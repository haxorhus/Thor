import os

def buscar_archivo(directorio, nombre_archivo):
    for raiz, directorios, archivos in os.walk(directorio):
        if nombre_archivo in archivos:
            return os.path.join(raiz, nombre_archivo)
    return None

def leer_coordenadas(archivo_txt):
    matriz = []
    
    with open(archivo_txt, 'r') as archivo:
        for linea in archivo:
            linea = linea.strip()
            if linea:
                coordenadas = linea.split()
                matriz.append([float(coord) for coord in coordenadas])
    
    return matriz

# Directorio donde quieres comenzar la búsqueda
directorio_busqueda = './input/'
nombre_archivo = 'coordenadas.txt'

# Buscar el archivo
ruta_archivo = buscar_archivo(directorio_busqueda, nombre_archivo)

if ruta_archivo:
    print(f"Archivo encontrado: {ruta_archivo}")
    matriz_coordenadas = leer_coordenadas(ruta_archivo)
    for fila in matriz_coordenadas:
        print(fila)
else:
    print("Archivo no encontrado.")
