# EPTPAC Simulation Code

This folder contains the MATLAB/Simulink code used to generate the simulation figures for the EPTPAC manuscript.

## Environment

- MATLAB with Simulink.
- The scripts were used with `sim_dt = 0.01`.
- Run scripts from this folder:

```matlab
cd E:\code\EPTPAC_GIT\EPTPAC\code
```

Generated figure files are written to subfolders under `code/`. Copy the final selected PDFs/PNGs to the manuscript `fig/` folder when needed.

## Main Files

- `InitialCondition.m`: defines the initial attitude, final attitude, inertia, torque/momentum limits, terminal time, total simulation time, and generates the reference trajectory using `Gauss_opt_mrp`.
- `Gauss_opt_mrp.m`: Gauss pseudospectral reference-trajectory optimization.
- `SildeModel_control.slx`: EPTPAC Simulink model.
- `SildeModel_control_compare.slx`: Ref. [19] comparison model.
- `savedata.m`: saves EPTPAC simulation data for Case 1/Case 2 time sweep.
- `savedata_precise.m`: saves data for prescribed-accuracy sweep.
- `savedata_PTC.m`: saves EPTPAC data used later in the Case 3 comparison.
- `result_plot.m`: plots Case 1 results.
- `tf_plot.m`: plots the Case 2 prescribed-time sweep.
- `precise_plot.m`: plots the Case 2 prescribed-accuracy sweep.
- `result_plot_compare.m`: plots Case 3 comparison between saved EPTPAC data and the current Ref. [19] workspace result.

## Data Folders

- `saved_data/`: EPTPAC data for Case 1 and Case 2 prescribed-time sweep.
- `saved_data_precise/`: EPTPAC data for Case 2 prescribed-accuracy sweep.
- `saved_data_PTCcompare/`: saved EPTPAC data for Case 3 comparison.
- `plots/`: Case 1 figures.
- `tf_comparison_plots/`: Case 2 prescribed-time figure.
- `precise_compare/`: Case 2 prescribed-accuracy figures.
- `PTC_compare/`: Case 3 comparison figures.

## Case 1: Nominal EPTPAC Tracking

1. Set the Case 1 parameters in `InitialCondition.m`, typically:

```matlab
tf = 120;
t_total = 150;
sim_dt = 0.01;
eps1 = 1e-5;
```

2. Run:

```matlab
InitialCondition
```

3. Run the EPTPAC Simulink model:

```matlab
sim('SildeModel_control')
```

4. Save the simulation data:

```matlab
savedata
```

5. Generate figures:

```matlab
result_plot
```

Figures are written to `plots/`.

## Case 2.1: Prescribed-Time Sweep

This case sweeps:

```matlab
tf_values = [110, 120, 130, 140];
```

For each `tf`:

1. Edit `InitialCondition.m`:

```matlab
tf = 110;      % then 120, 130, 140
t_total = 150;
sim_dt = 0.01;
eps1 = 1e-5;
```

2. Run:

```matlab
InitialCondition
sim('SildeModel_control')
savedata
```

3. After all four `tf` cases are saved, generate the sweep figure:

```matlab
tf_plot
```

The current `tf_plot.m` generates `qe_norm_comparison.pdf/png` in `tf_comparison_plots/`, showing `||sigma_{e,tar}||` for different prescribed terminal times.

## Case 2.2: Prescribed-Accuracy Sweep

This case sweeps:

```matlab
eps1_list = [1e-4, 1e-5, 1e-6, 1e-7];
```

For each `eps1`:

1. Edit `InitialCondition.m`:

```matlab
tf = 120;
t_total = 120;
sim_dt = 0.01;
eps1 = 1e-4;   % then 1e-5, 1e-6, 1e-7
```

2. Run the EPTPAC simulation:

```matlab
InitialCondition
sim('SildeModel_control')
savedata_precise
```

3. After all four accuracy cases are saved, generate figures:

```matlab
precise_plot
```

Figures are written to `precise_compare/`:

- `CaseSweep_MRPErrorNorm_log10.pdf`
- `CaseSweep_SlidingNorm_log10.pdf`

## Case 3: Comparison With Ref. [19]

Case 3 uses a two-stage workflow:

1. Save the EPTPAC data.
2. Run the Ref. [19] model in the current MATLAB workspace.
3. Run the comparison plotting script.

### Stage 1: Save EPTPAC Data

Run the EPTPAC model for each Case 3 setting and save it using:

```matlab
savedata_PTC
```

This writes EPTPAC torque and wheel-momentum data to `saved_data_PTCcompare/`:

- `tau_t300.mat`, `hw_t300.mat`
- `tau_t400.mat`, `hw_t400.mat`
- `tau_t500.mat`, `hw_t500.mat`

The saved files use `sim_dt = 0.01`.

### Stage 2: Run Ref. [19]

Run the Ref. [19] comparison model:

```matlab
sim('SildeModel_control_compare')
```

The current MATLAB workspace should then contain the Ref. [19] result variables such as `qi`, `wb`, `RWT`, `HW`, `qe`, and `Omegae`.

### Stage 3: Plot Comparison

Set `tf` and `t_total`, then run:

```matlab
result_plot_compare
```

Use the following settings:

```matlab
% Case 3.1, manuscript Tf = 300 s
tf = 300;
t_total = 400;
result_plot_compare

% Case 3.1, manuscript Tf = 150 s
tf = 150;
t_total = 300;
result_plot_compare

% Case 3.2, dual saturation
tf = 300;
t_total = 500;
result_plot_compare
```

The plotting script reads saved EPTPAC data from `saved_data_PTCcompare/` and overlays the current Ref. [19] workspace data. The Ref. [19] vertical marker is set automatically:

- `t_total = 300`: Ref. [19] marker at 260 s.
- `t_total = 400`: Ref. [19] marker at 170 s.
- `t_total = 500`: Ref. [19] marker at 410 s.

Figures are written to `PTC_compare/`.

## Notes

- If a plotting script warns that workspace data are shorter than `0:sim_dt:t_total`, rerun the corresponding Simulink model with the intended `t_total`.
- `result_plot_compare.m` expects saved EPTPAC data and current Ref. [19] workspace data.
- `tf_plot.m` and `precise_plot.m` do not modify manuscript files. They only regenerate figure files.
- The manuscript `main.tex` should be updated only after final figures are selected and copied to `fig/`.
