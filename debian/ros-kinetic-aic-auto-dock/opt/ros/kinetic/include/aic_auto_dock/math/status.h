#ifndef STATUS_H
#define STATUS_H

struct ModuleData
{
  enum ModuleStep
  {
    Empty,
    RecognizeStep,
    DockingStep,
    GetOutStep,
    SuccessFinsh
  };

  ModuleStep currentStep;
  bool current_status;
  double remaining_distance;
};

enum actionlibStatus
{
  executing,
  paused
};

enum docking_direction
{
  backward,
  forward,
  rotate,
  still
};

enum status
{
  empty,
  timeout,
  success,
  failed,
  canclegoal,
  uncertain,
  emergencybuttom
};

struct StepProcess
{
  enum Process
  {
    prepareForceStep,
    prepareStep,
    portStep
  };
  Process process;

  bool prepareNavStepProcessing = false;
  bool prepareNavStepResult = false;
  bool prepareNavStepInit = false;

  bool prepareStepProcessing = false;
  bool prepareStepResult = false;
  bool prepareStepInit = false;

  bool portStepProcessing = false;
  bool portStepResult = false;
  bool portStepInit = false;
};

#endif // STATUS_H
