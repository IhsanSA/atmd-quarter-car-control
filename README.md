# Active Tuned Mass Damper Control for a Quarter-Car Model Using PID and LQR

This project investigates an **Active Tuned Mass Damper (ATMD)** to reduce vertical body displacement in a passenger-car quarter-car model. Inspired by the mass damper concept used in motorsport, it examines how an auxiliary mass and an actuator reduce vibration caused by road disturbances.

The study compares a standard vehicle, a passive Tuned Mass Damper (TMD), an ATMD with a Proportional–Integral–Derivative (PID) controller, and an ATMD with a Linear Quadratic Regulator (LQR). It combines time-domain simulation, frequency-domain analysis, and Monte Carlo sensitivity analysis, while considering actuator force, absorber travel, and mechanical end-stop contact.

For a 4 mm road wave at 1.2 Hz, PID and LQR reduced peak body displacement by 11.41% and 10.53%, respectively, compared with the passive TMD. At comparable reductions, LQR used approximately 14% less RMS actuator force than PID.

> This is a simulation study. Results depend on the model parameters, road inputs, controller settings, and actuator assumptions.

## ⚙️ Project Process

### 1. 🚗 Mathematical Modeling

- Developed a vertical quarter-car model with vehicle body, wheel, and auxiliary damper masses.
- Derived equations of motion using Newton’s second law and represented the suspension and tire with spring–damper elements.
- Connected the auxiliary mass to the body through a spring, damper, and actuator. The actuator applies equal and opposite forces to the body and auxiliary mass.
- Defined body displacement relative to static equilibrium.

### 2. 🛣️ Road Disturbance Modeling

- Used a smooth single bump to examine transient response and settling.
- Used sinusoidal roads to examine the effect of excitation frequency.
- Examined a processed measured-road elevation segment after checking distances, elevations, sample spacing, and suspicious changes.
- Specified vehicle speed when converting spatial road profiles to time-dependent inputs.

### 3. 🎛️ Control Strategies

- **Standard vehicle** — quarter-car model without an auxiliary damper.
- **Passive TMD** — tuned auxiliary mass connected by a spring and damper.
- **ATMD-PID** — body displacement and velocity feedback with anti-windup.
- **ATMD-LQR** — state feedback using the positions and velocities of all three masses.

Tuning and evaluation scenarios were separated. Controller gains remained fixed during evaluation and parameter-variation tests.

### 4. 📈 Time-Domain Analysis

The configurations used identical road inputs and initial conditions. Metrics included peak and RMS body displacement, peak and RMS actuator force, maximum relative travel, end-stop contact, and settling behavior.

### 5. 📉 Frequency-Domain Analysis

State-space models and Bode plots were used to compare road-to-body magnitude and phase. In the nominal linear model, road-to-body gain was lower than the passive TMD over approximately **0.89–1.70 Hz for PID** and **0.76–1.94 Hz for LQR**. These ranges depend on the specified linear model and parameters.

### 6. 🎲 Monte Carlo Sensitivity Analysis

- Varied seven input factors, including road amplitude, excitation frequency, vehicle parameters, and damper travel.
- Evaluated 9,216 parameter sets across three damper configurations, for 27,648 simulations, using a fixed random seed.
- Estimated total-order Sobol indices for selected outputs.
- Within the selected ranges, road amplitude had the greatest influence on peak body displacement. Excitation frequency had the greatest influence on additional peak reduction relative to the passive TMD.

### 7. ⚡ Actuator and Physical Constraints

The model includes actuator force limits, continuous-force and speed screening, and end stops engaging at ±40 mm relative travel. It also examines sensitivity to contact stiffness and damping. Electrical input power uses an assumed 95% motoring efficiency. Absorbed mechanical energy is recorded separately without regeneration credit.

### 8. ✅ Model Verification

Selected model responses were compared with published reference cases. The Monte Carlo simulator was checked against individual time-history simulations. Reference-model verification is separate from evaluation of the project's ATMD design.

## 📊 Key Results

### Nominal Sinusoidal Road

For a **4 mm road amplitude at 1.2 Hz**:

| Configuration | Peak body displacement | RMS actuator force |
|---|---:|---:|
| Standard vehicle | 7.64 mm | — |
| Passive TMD | 6.88 mm | — |
| ATMD-PID | 6.10 mm | 3.51 N |
| ATMD-LQR | 6.16 mm | 3.02 N |

Compared with the passive TMD, PID reduced the peak by 11.41% and LQR by 10.53%. LQR required approximately 14% less RMS actuator force for these comparable reductions.

### Effect of the Travel Limit

At 30 mm road amplitude and 1.2 Hz, end-stop contact affected the response. PID produced a peak 1.60% higher than the passive TMD, while LQR retained a 0.70% reduction. The contact behavior is model-dependent and requires physical validation.

## 📚 What I Learned

- Developed a quarter-car model using Newton’s laws and state-space representation.
- Compared passive damping, PID, and LQR active control and evaluated actuator force trade-offs.
- Used time-domain simulations, Bode plots, Monte Carlo simulation, and Sobol indices to study performance and sensitivity.
- Investigated actuator limits and mechanical contact.
- Separated controller tuning from evaluation and examined simulation-model limitations.

## 🚀 How It Can Be Improved

- Validate the model against measured vehicle suspension responses.
- Refine the actuator model using measured force–velocity and thermal characteristics.
- Develop a sensor and state-estimation model for LQR.
- Extend the quarter-car model to include pitch and roll.
- Evaluate additional measured roads and control approaches.
- Test the controller using hardware-in-the-loop simulation.

## 💻 Tools and Project Files

- Python, NumPy, SciPy, Pandas, Matplotlib, and Google Colab.
- [Simulation notebook](ATMD_Quarter_Car_Colab.ipynb)
- [Research paper](Research/Paper_ATMD_PID_LQR_English_Final.pdf)
- [Technical case study poster (PDF)](ATMD_Portfolio_Poster.pdf)
- [Editable poster (PowerPoint)](ATMD_Portfolio_Poster.pptx)

## ▶️ How to Run

1. Open the notebook in Google Colab.
2. Select **Runtime → Run all**.
3. Review the model parameters, road scenarios, controller settings, and generated results.
4. Change inputs and rerun relevant cells to explore other scenarios.

The notebook includes an embedded measured-road segment. The complete CSV is needed to select other sections.
