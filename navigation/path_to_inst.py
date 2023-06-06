import numpy as np

""" CONSTANTES DE PRUEBA """
NUM_QUIETOS = 15 # Número de veces que se repite QUIETO entre instrucciones
FACTOR_TRIGGERR = 3.5 # Factor por el que se divide el número de veces que se repite TriggerR entre instrucciones
NUM_GIROS_90G = 20 # Número de veces que se repite una instruccion de giro para girar 90 grados

def get_coordinates(coords_path):
    with open(coords_path, "r") as f:
        f.readline() # Skip the first line
        data = []
        for line in f:
            data.append(line.strip().split(","))

    return np.array(data, dtype=np.uint8)

def get_instructions(data):
    """
    TriggerR: Ir Adelante
    TriggerL: Ir Atras
    Derecha: Girar Derecha
    Izquierda: Girar Izquierda
    """
    instructions = ["TriggerR"]

    y1, x1 = data[0]
    y2, x2 = data[1]

    if x2 == x1 and y2 > y1:
        state = "y"
    elif x2 == x1 and y2 < y1:
        state = "-y"
    elif x2 > x1 and y2 == y1:
        state = "x"
    elif x2 < x1 and y2 == y1:
        state = "-x"

    for i in range(2, data.shape[0] - 1):
        y1, x1 = data[i]
        y2, x2 = data[i + 1]

        if state == "y":
            if x2 < x1:
                v = "Derecha"
                state = "-x"
            elif x2 > x1:
                v = "Izquierda"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "-y":
            if x2 < x1:
                v = "Izquierda"
                state = "-x"
            elif x2 > x1:
                v = "Derecha"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "x":
            if y2 < y1:
                v = "Izquierda"
                state = "-y"
            elif y2 > y1:
                v = "Derecha"
                state = "y"
            else:
                v = "TriggerR"
        elif state == "-x":
            if y2 < y1:
                v = "Derecha"
                state = "-y"
            elif y2 > y1:
                v = "Izquierda"
                state = "y"
            else:
                v = "TriggerR"

        # Si el último movimiento guardado no es TriggerR y el movimiento a guardar tampoco, entonces se elimina el último movimiento guardado y se guarda el nuevo movimiento
        if instructions[-1] != "TriggerR" and v != "TriggerR":
            instructions.pop()
            instructions.append(v)
        else:
            instructions.append(v)

    return instructions

def save_instructions(instructions, instructions_path):
    with open(instructions_path, "w") as f:
        count, rect = 0, True
        for instruction in instructions:
            if instruction == "TriggerR":
                count += 1
            else:
                rect = False
                f.write(("QUIETO\n")*NUM_QUIETOS)
                f.write(("TriggerR\n")*(int(count//(FACTOR_TRIGGERR))))
                f.write(("QUIETO\n")*NUM_QUIETOS)
                count = 0
                if instruction == "Derecha":
                    instruction = "Izquierda"
                elif instruction == "Izquierda":
                    instruction = "Derecha"
                f.write((instruction + "\n")*NUM_GIROS_90G)
        
        if rect:
            f.write(("QUIETO\n")*NUM_QUIETOS)
            f.write(("TriggerR\n")*(int(count//(FACTOR_TRIGGERR))))
            f.write(("QUIETO\n")*NUM_QUIETOS)

def to_instructions(coords_path, instructions_path):
    print(coords_path, instructions_path)
    data = get_coordinates(coords_path)
    instructions = get_instructions(data)
    save_instructions(instructions, instructions_path)

if __name__ == "__main__":
    to_instructions("path.txt", "instructions.txt")
