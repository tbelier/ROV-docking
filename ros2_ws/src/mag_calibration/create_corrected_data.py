import numpy as np

# Fonction pour lire les vecteurs à partir du fichier
def lire_vecteurs(nom_fichier):
    vecteurs = []
    with open(nom_fichier, 'r') as file:
        for line in file:
            vecteur = list(map(float, line.strip().split()))
            vecteurs.append(vecteur)
    return vecteurs

# Fonction pour appliquer le calcul matriciel aux vecteurs
def calcul_matriciel(vecteurs):
    A = np.array([
        [1.074043508698078409e+00, -1.825169088465639078e-02, -2.506236595745721663e-01],
        [-1.825169088465639078e-02, 9.203150980722383245e-01, 9.558293574412760063e-02],
        [-2.506236595745722218e-01, 9.558293574412757287e-02, 1.079564619015944915e+00]]) 
    b = np.array([[-1.782746475458791906e+02, 1.906317241941198972e+02, 6.746973582435566641e+02]]).T
    vecteurs_transformes = []
    for vecteur in vecteurs:
        vecteur = np.array(vecteur)
        vecteur = np.reshape(vecteur, (3,1))
        vecteur_transforme = A@vecteur -b
        vecteurs_transformes.append(vecteur_transforme)
    return vecteurs_transformes

# Fonction pour écrire les résultats dans un nouveau fichier
def ecrire_resultats(nom_fichier, resultats):
    with open(nom_fichier, 'w') as file:
        for resultat in resultats:
            file.write(' '.join(str(x[0]) for x in resultat) + '\n')

# Utilisation des fonctions
nom_fichier_entree = 'raw_data_magneto_ROV_11_24.txt'  # Remplace avec le nom de ton fichier d'entrée
nom_fichier_sortie = 'raw_data_magneto_ROV_11_24_corrected.txt'  # Nom du fichier de sortie

vecteurs = lire_vecteurs(nom_fichier_entree)
resultats = calcul_matriciel(vecteurs)
ecrire_resultats(nom_fichier_sortie, resultats)