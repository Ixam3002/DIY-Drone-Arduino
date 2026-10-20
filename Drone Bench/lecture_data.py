import matplotlib.pyplot as plt
import pandas as pd

# 1. Chargement des données
name = "pitch_kp0.75,ki0.0,kd0.0"

df = pd.read_csv(name + '.csv')

# Conversion du timestamp en format datetime et définition comme index temporaire
df['timestamp'] = pd.to_datetime(df['timestamp'])
df = df.set_index('timestamp')

# 2. Interpolation linéaire temporelle (fonctionne car l'index est en datetime)
df_interpolated = df.interpolate(method='time')

# 3. Rééchantillonnage (Downsampling) : un point toutes les 20 millisecondes
df_resampled = df_interpolated.resample('20ms').mean()

# 4. Calculer le temps écoulé en secondes depuis le premier enregistrement pour l'axe X
secondes_depuis_debut = (df_resampled.index - df_resampled.index.min()).total_seconds()

# 5. Création des graphiques
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Graphique 1 : Pitch et Target Pitch
axes[0].plot(
    secondes_depuis_debut,
    df_resampled['pitch'],
    label='Pitch mesuré',
    color='blue',
    linewidth=1.5,
)
axes[0].plot(
    secondes_depuis_debut,
    df_resampled['target_pitch'],
    label='Target Pitch',
    color='orange',
    linestyle='--',
    linewidth=1.5,
)
axes[0].set_ylabel('Pitch (deg)')
axes[0].set_title('Pitch et Target Pitch (Interpolé & Rééchantillonné)')
axes[0].legend(loc='upper right')
axes[0].grid(True)

# Graphique 2 : Roll et Target Roll
axes[1].plot(
    secondes_depuis_debut,
    df_resampled['roll'],
    label='Roll mesuré',
    color='green',
    linewidth=1.5,
)
axes[1].plot(
    secondes_depuis_debut,
    df_resampled['target_roll'],
    label='Target Roll',
    color='red',
    linestyle='--',
    linewidth=1.5,
)
axes[1].set_ylabel('Roll (deg)')
axes[1].set_title('Roll et Target Roll (Interpolé & Rééchantillonné)')
axes[1].legend(loc='upper right')
axes[1].grid(True)

# Graphique 3 : Les 4 moteurs (ESC)
axes[2].plot(
    secondes_depuis_debut, df_resampled['escA'], label='ESC A', linewidth=1.2
)
axes[2].plot(
    secondes_depuis_debut, df_resampled['escB'], label='ESC B', linewidth=1.2
)
axes[2].plot(
    secondes_depuis_debut, df_resampled['escC'], label='ESC C', linewidth=1.2
)
axes[2].plot(
    secondes_depuis_debut, df_resampled['escD'], label='ESC D', linewidth=1.2
)
axes[2].set_ylabel('Commandes ESC')
axes[2].set_xlabel('Temps depuis le début (secondes)')
axes[2].set_title('Commandes des moteurs (ESC)')
axes[2].legend(loc='upper right')
axes[2].grid(True)
axes[2].set_ylim(1000, 2000)

# Mise en page et affichage
plt.tight_layout()
plt.savefig(name + '.png', dpi=300)
plt.show()