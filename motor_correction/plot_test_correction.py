import numpy as np
import matplotlib.pyplot as plt

def get_polynomial_coefficients(x_list, y_list, deg=2):
    """
    Computes the coefficients of a polynomial fit for given x and y lists.
    """
    x = np.array(x_list, dtype=float)
    y = np.array(y_list, dtype=float)
    
    # Calculate polynomial coefficients (least squares fit)
    coefficients = np.polyfit(x, y, deg)
    return coefficients

# Your data (Note: y_list has 13 elements, so x_list is scaled to match)
y_list = np.array([0.0, 0.1, 0.3, 0.4, 0.5, 0.6, 0.6, 0.7, 0.7, 0.6, 0.75, 0.95])
x_list = np.array([1000,1100,1150,1200,1300,1400,1500,1600,1700,1800,1900,2000])


esc_motA= np.array([1000,1100,1150,1200,1300,1400,1500,1600,1700,1800,1900,2000])
force_motA = np.array([0.00, 0.00, 0.08, 0.22, 0.5, 0.77, 0.91, 1.28, 1.5, 1.85, 2.43, 3.05])
esc_motB = np.array([1000,1040, 1050, 1060, 1070, 1080, 1090, 1100,1150,1200,1300,1400,1500,1600,1700,1800,1900,2000])
force_motB = np.array([0.00, 0.05, 0.1, 0.15, 0.2, 0.25, 0.29, 0.36, 0.44, 0.60, 0.87, 1.14, 1.70, 1.90, 2.35, 3.23, 3.23, 3.23])

corr_motB = np.array([0.0, 0.15, 0.25, 0.35, 0.5, 0.63, 0.68, 0.7, 0.70, 0.76, 0.78, 0.78])
esc_corr_motB = np.array(1000 + (esc_motA-1000) * corr_motB)

print(esc_corr_motB)

# Get coefficients and build polynomial function
coeffs_motA = get_polynomial_coefficients(esc_motA, force_motA, deg=2)
coeffs_motB = get_polynomial_coefficients(esc_motB, force_motB, deg=9)
p_motA = np.poly1d(coeffs_motA)
p_motB = np.poly1d(coeffs_motB)

def puissance_motB(esc):
    mask = esc < 1800

    force = np.concatenate([p_motB(esc[mask]), [3.23 for _ in range(np.sum(esc>=1800))]])

    return force

# Generate smooth points for the curve line
x_smooth = np.linspace(esc_motA.min(), esc_motA.max(), 200)
x_smooth_motB_corr = np.linspace(esc_corr_motB.min(), esc_corr_motB.max(), 200)
smooth_force_motA = p_motA(x_smooth)
smooth_force_motB = puissance_motB(x_smooth)
smooth_force_motB_corr = puissance_motB(x_smooth_motB_corr)
force_motB_corr = puissance_motB(esc_corr_motB)


# Plotting the comparison
plt.figure(figsize=(9, 6))
plt.scatter(esc_motA, force_motA, color='red', s=50, label='Mot A', zorder=5)
plt.scatter(esc_motB, force_motB, color='blue', s=50, label='Mot B', zorder=5)
plt.scatter(esc_motA, force_motB_corr, color='black', s=50, label='Mot B', zorder=5)
plt.plot(x_smooth, smooth_force_motA, color='red', linestyle='--', linewidth=2, label=f'mot A')
plt.plot(x_smooth, smooth_force_motB, color='blue', linestyle='--', linewidth=2, label=f'mot B')
# plt.plot(x_smooth, smooth_force_motB_corr, color='black', linestyle='--', linewidth=2, label=f'mot B corr')


plt.title('Polynomial Interpolation vs. Raw Data Points', fontsize=14, fontweight='bold')
plt.xlabel('PWM Power (x)', fontsize=12)
plt.ylabel('Force / Value (y)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(fontsize=11)
plt.tight_layout()
plt.show()



#Function correction motor
coeff_corr = get_polynomial_coefficients(esc_motA, corr_motB, deg=5)
print(coeff_corr)
p_correc = np.poly1d(coeff_corr)

esc = np.linspace(1000, 2000, 200)
corr = p_correc(esc)

plt.figure()
plt.plot(esc, corr, label="curve correction")
plt.scatter(esc_motA, corr_motB, label = "Raw values")
plt.title("function correction motor B")
plt.xlabel('PWM Power', fontsize=12)
plt.ylabel('Correction', fontsize=12)
plt.grid()
plt.legend()
plt.show()
