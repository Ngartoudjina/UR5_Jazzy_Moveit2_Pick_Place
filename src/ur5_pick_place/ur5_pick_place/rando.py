import random

def creer_groupes(prenoms):
    if len(prenoms) != 18:
        raise ValueError("Il faut exactement 18 prénoms.")

    # Mélanger aléatoirement
    random.shuffle(prenoms)

    # Créer 6 groupes de 3
    groupes = [prenoms[i:i+3] for i in range(0, 18, 3)]

    return groupes


def afficher_groupes(groupes):
    for i, groupe in enumerate(groupes, start=1):
        print(f"Groupe {i} : {', '.join(groupe)}")


if __name__ == "__main__":
    print("Entrez 18 prénoms (un par ligne) :")
    
    prenoms = []
    for i in range(18):
        prenom = input(f"Prénom {i+1} : ")
        prenoms.append(prenom.strip())

    groupes = creer_groupes(prenoms)
    
    print("\nGroupes générés :\n")
    afficher_groupes(groupes)