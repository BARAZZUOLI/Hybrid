import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import os  # Import ajouté pour vérifier l'existence du fichier

def lire_points_fichier(nom_fichier):
    """
    Lit les coordonnées x et y à partir d'un fichier texte.
    Format attendu : une paire x,y par ligne
    Exemple :
    0,0
    300,500
    700,500
    1000,0
    """
    x_points = []
    y_points = []
    
    # Vérifier d'abord si le fichier existe
    if not os.path.exists(nom_fichier):
        print(f"✗ ERREUR CRITIQUE: Le fichier '{nom_fichier}' n'existe pas!")
        print(f"  Chemin absolu essayé: {os.path.abspath(nom_fichier)}")
        print(f"  Répertoire courant: {os.getcwd()}")
        print("  Liste des fichiers .txt disponibles:")
        for f in os.listdir('.'):
            if f.endswith('.txt'):
                print(f"    - {f}")
        print("\n  Utilisation des valeurs par défaut...")
        x_points = [0, 300, 700, 1000]
        y_points = [0, 500, 500, 0]
        return x_points, y_points
    
    try:
        print(f"✓ Tentative de lecture du fichier: '{nom_fichier}'")
        print(f"  Taille du fichier: {os.path.getsize(nom_fichier)} octets")
        
        with open(nom_fichier, 'r') as fichier:
            lignes = fichier.readlines()
            print(f"  Nombre de lignes dans le fichier: {len(lignes)}")
            
            for ligne_num, ligne in enumerate(lignes, 1):
                ligne = ligne.strip()
                if not ligne or ligne.startswith('#'):
                    print(f"  Ligne {ligne_num}: [ignorée - vide ou commentaire]")
                    continue
                
                print(f"  Ligne {ligne_num}: '{ligne}'")
                
                # Essayer différents séparateurs
                if ',' in ligne:
                    try:
                        x_str, y_str = ligne.split(',')
                    except ValueError:
                        print(f"    ERREUR: Impossible de découper par virgule")
                        continue
                elif ';' in ligne:
                    try:
                        x_str, y_str = ligne.split(';')
                    except ValueError:
                        print(f"    ERREUR: Impossible de découper par point-virgule")
                        continue
                elif '\t' in ligne:
                    try:
                        x_str, y_str = ligne.split('\t')
                    except ValueError:
                        print(f"    ERREUR: Impossible de découper par tabulation")
                        continue
                else:
                    parts = ligne.split()
                    if len(parts) >= 2:
                        x_str, y_str = parts[0], parts[1]
                    else:
                        print(f"    ERREUR: Moins de 2 éléments dans la ligne")
                        continue
                
                try:
                    x_val = float(x_str.strip())
                    y_val = float(y_str.strip())
                    x_points.append(x_val)
                    y_points.append(y_val)
                    print(f"    ✓ Ajouté: ({x_val}, {y_val})")
                except ValueError as ve:
                    print(f"    ERREUR de conversion en float: {ve}")
                    continue
        
        print(f"\n✓ Fichier '{nom_fichier}' lu avec succès")
        print(f"  Points chargés: {len(x_points)}")
        
        if len(x_points) == 0:
            print("  ATTENTION: Aucun point valide trouvé dans le fichier!")
            print("  Utilisation des valeurs par défaut...")
            x_points = [0, 300, 700, 1000]
            y_points = [0, 500, 500, 0]
            return x_points, y_points
        
        # Ajout automatique du premier point à la fin pour fermer la forme
        if len(x_points) > 2 and (x_points[0] != x_points[-1] or y_points[0] != y_points[-1]):
            x_points.append(x_points[0])
            y_points.append(y_points[0])
            print(f"  Point de fermeture ajouté automatiquement")
            
    except Exception as e:
        print(f"✗ Erreur lors de la lecture du fichier: {type(e).__name__}: {e}")
        print("  Utilisation des valeurs par défaut...")
        x_points = [0, 300, 700, 1000]
        y_points = [0, 500, 500, 0]
    
    return x_points, y_points

# ==============================
# LECTURE DES DONNÉES DEPUIS FICHIER
# ==============================
print("="*60)
print("DÉBUT DU PROGRAMME")
print("="*60)

# Nom du fichier contenant les coordonnées
nom_fichier = "x_z_data.txt"  # Changez ce nom selon votre fichier

# Lecture des points depuis le fichier
print(f"\nTentative de lecture du fichier: '{nom_fichier}'")
x, y = lire_points_fichier(nom_fichier)

# Affichage des points chargés
print("\n" + "="*60)
print("RÉSUMÉ DES POINTS CHARGÉS:")
print("="*60)
for i, (xi, yi) in enumerate(zip(x, y)):
    print(f"  Point {i}: ({xi}, {yi})")

# Si seulement 1 point a été chargé, c'est qu'il y a un problème
if len(x) < 3:
    print(f"\n⚠️  ATTENTION: Seulement {len(x)} points chargés!")
    print("   Cela peut indiquer un problème avec le fichier.")
    print("   Vérifiez le format du fichier (ex: 0,0 sur chaque ligne)")
    print("   Utilisation des valeurs par défaut du trapèze...")
    x = [0, 300, 700, 1000]
    y = [0, 500, 500, 0]

# Interpolation pour une animation fluide
t = np.linspace(0, 1, 300)  # Augmenté pour plus de fluidité
x_traj, y_traj = [], []

for i in range(len(x)-1):
    seg = np.linspace(0, 1, 75)  # Plus de points par segment
    x_traj.extend(x[i] + (x[i+1] - x[i]) * seg)
    y_traj.extend(y[i] + (y[i+1] - y[i]) * seg)

# Conversion en arrays numpy
x_traj = np.array(x_traj)
y_traj = np.array(y_traj)

print(f"\n" + "="*60)
print("CRÉATION DE L'ANIMATION")
print("="*60)
print(f"Nombre de points de trajectoire: {len(x_traj)}")
print(f"Nombre de segments: {len(x)-1}")

# Création de la figure avec les BONNES LIMITES
fig, ax = plt.subplots(figsize=(12, 8))

# Ajustement automatique des limites avec une marge
margin_x = 0.1 * (max(x) - min(x))  # 10% de marge
margin_y = 0.1 * (max(y) - min(y))
ax.set_xlim(min(x) - margin_x, max(x) + margin_x)
ax.set_ylim(min(y) - margin_y, max(y) + margin_y)

# Configuration du graphique
ax.grid(True, alpha=0.3)
ax.set_aspect('equal', adjustable='datalim')  # Conserve les proportions
ax.set_xlabel('Position X', fontsize=12)
ax.set_ylabel('Position Y', fontsize=12)
ax.set_title(f'Animation de la trajectoire - Fichier: {nom_fichier}', fontsize=14, fontweight='bold')

# Ajout des étiquettes pour les points
for i, (xi, yi) in enumerate(zip(x[:-1], y[:-1])):
    nom_point = chr(65 + i) if i < 26 else f'P{i}'  # A, B, C, ... ou P1, P2, ...
    ax.text(xi, yi + 0.05 * margin_y, f'Point {nom_point}', 
            ha='center', va='bottom', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

# Éléments d'animation
point, = ax.plot([], [], 'ro', markersize=12, 
                 markerfacecolor='red', markeredgecolor='darkred',
                 markeredgewidth=2, zorder=5, label='Position actuelle')
trace, = ax.plot([], [], 'b-', linewidth=3, alpha=0.8, zorder=3, label='Trajectoire')
polygone_line, = ax.plot(x, y, 'k--', alpha=0.4, linewidth=1.5, zorder=2, label='Forme du polygone')
polygone_points, = ax.plot(x[:-1], y[:-1], 'ks', markersize=8, 
                          markerfacecolor='yellow', zorder=4, label='Points de contrôle')

# Ajout d'une légende
ax.legend(loc='upper right')

# Texte d'information
info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                   fontsize=10, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

def animate(i):
    # Mise à jour du point mobile
    point.set_data([x_traj[i]], [y_traj[i]])
    
    # Mise à jour de la trace (trajectoire parcourue)
    trace.set_data(x_traj[:i+1], y_traj[:i+1])
    
    # Mise à jour des informations
    progression = 100 * (i+1) / len(x_traj)
    info_text.set_text(f'Position: ({x_traj[i]:.0f}, {y_traj[i]:.0f})\n'
                      f'Progression: {progression:.1f}%\n'
                      f'Frame: {i+1}/{len(x_traj)}\n'
                      f'Points: {len(x)-1} sommets')
    
    return point, trace, info_text

# Création de l'animation
ani = FuncAnimation(fig, animate, frames=len(x_traj), 
                    interval=20, blit=True, repeat=True)

# Nom du fichier de sortie basé sur le fichier d'entrée
nom_sortie = nom_fichier.replace('.txt', '_animation.gif')
if nom_sortie == nom_fichier:  # Si pas d'extension .txt
    nom_sortie = f"{nom_fichier}_animation.gif"

# Sauvegarde en GIF
print(f"\n" + "="*60)
print("GÉNÉRATION DU GIF")
print("="*60)
print(f"Fichier source: {nom_fichier}")
print(f"Fichier sortie: {nom_sortie}")
print(f"Dimensions de la forme:")
print(f"  Largeur: {max(x) - min(x):.1f} unités")
print(f"  Hauteur: {max(y) - min(y):.1f} unités")
print(f"  Nombre de frames: {len(x_traj)}")
print(f"  Nombre de sommets: {len(x)-1}")

try:
    ani.save(nom_sortie, 
             writer=PillowWriter(fps=30, bitrate=1800),
             dpi=100,
             progress_callback=lambda i, n: print(f"  Frame {i+1}/{n}") 
             if i % 50 == 0 else None)
    
    print(f"\n✓ GIF sauvegardé avec succès: {nom_sortie}")
    print(f"  Résolution: {fig.get_size_inches()[0]*100:.0f}x{fig.get_size_inches()[1]*100:.0f} pixels")
    print(f"  Durée: {len(x_traj)/30:.1f} secondes")
    print(f"  FPS: 30")
    
except Exception as e:
    print(f"\n✗ ERREUR lors de la sauvegarde du GIF: {e}")
    print("  Tentative de sauvegarde avec un nom différent...")
    try:
        nom_sortie_secours = "animation_sauvegarde.gif"
        ani.save(nom_sortie_secours,
                 writer=PillowWriter(fps=30, bitrate=1800),
                 dpi=100)
        print(f"✓ GIF sauvegardé sous: {nom_sortie_secours}")
    except Exception as e2:
        print(f"✗ ERREUR finale: Impossible de sauvegarder le GIF: {e2}")

# Calcul et affichage des informations géométriques
print(f"\n" + "="*60)
print("CARACTÉRISTIQUES GÉOMÉTRIQUES")
print("="*60)
print(f"  Périmètre approximatif: {np.sum(np.sqrt((np.diff(x))**2 + (np.diff(y))**2)):.1f} unités")

# CORRECTION : Calcul de l'aire (formule du shoelace) - Version corrigée
aire = 0.5 * abs(sum(x[i] * y[i+1] - x[i+1] * y[i] for i in range(len(x)-1)))
print(f"  Aire: {aire:.1f} unités²")

# Information supplémentaire sur la forme
if len(x) == 5:
    print("  Forme: Trapèze fermé")
elif len(x) == 4:
    print("  Forme: Triangle fermé")
elif len(x) > 5:
    print(f"  Forme: Polygone à {len(x)-1} côtés")
else:
    print("  Forme: Ligne ouverte")

print(f"\n" + "="*60)
print("INSTRUCTIONS POUR CRÉER LE FICHIER DE POINTS")
print("="*60)
print("Créez un fichier 'x_z_data.txt' avec ce format:")
print("  0,0")
print("  300,500")
print("  700,500")
print("  1000,0")
print("\nOu utilisez cet exemple rapide:")
print(f"Écho vers le fichier: echo -e '0,0\\n300,500\\n700,500\\n1000,0' > x_z_data.txt")

# Créer un fichier d'exemple s'il n'existe pas
if not os.path.exists("exemple_points.txt"):
    with open("exemple_points.txt", "w") as f:
        f.write("# Exemple de fichier de points\n")
        f.write("# Format: x,y\n")
        f.write("0,0\n")
        f.write("300,500\n")
        f.write("700,500\n")
        f.write("1000,0\n")
    print(f"\n✓ Fichier exemple créé: 'exemple_points.txt'")

plt.tight_layout()
plt.show()

print(f"\n" + "="*60)
print("FIN DU PROGRAMME")
print("="*60)
