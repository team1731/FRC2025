package frc.robot.state;

import java.lang.reflect.Method;
import java.util.HashMap;
import frc.robot.commands.CommandCallback;


public abstract class StateMachine {
  private Object stateTransitionTable[][];
  private HashMap<String, Method> methods;
  protected State currentState;
  protected CommandCallback commandCallback;

  // will be passed to subsystems to receive feedback for state transitions
  protected StateMachineCallback inputCallback = (Input input) -> {
    run(input);
  };

  public abstract void endSequence();
  
  public void setCallback(CommandCallback callback) {
    commandCallback = callback;
  }

  public void setInput(Input input) {
    run(input);
  }

  protected void processComplete() {
    if(commandCallback != null) {
      commandCallback.processComplete();
    }
  }

  protected void setCurrentState(State state) {
    currentState = state;
  }

  protected void setStateTransitionTable(Object[][] table) {
    stateTransitionTable = table;
    methods = new HashMap<String, Method>();

    // cycle through state transition table and pre-load the operation methods 
    for(Object[] transition : stateTransitionTable) {
      String methodName = (String)transition[2];
      try {
        // not every state transition will involve an operation, null is valid here
        if(methodName != null) {
          Method method = this.getClass().getMethod(methodName);
          if(method != null) {
            methods.put(methodName, method);
          }
        }
      } catch (NoSuchMethodException e) {
        System.out.println(" ** ERROR: method '" + methodName + "()' NOT FOUND IN CLASS " + getClass().getSimpleName() + "! ** ");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  private void run(Input input) {
    Object[] operationAndNextState = lookupOperationAndNextState(currentState, input);
    if(operationAndNextState == null) {
      // TODO log an error, no match found
      return;
    }

    String methodName = (String)operationAndNextState[0];
    Method method = methods.get(methodName);
    State nextState = (State) operationAndNextState[1];
    
    System.out.println(
      getClass().getSimpleName() + 
      " - received input: " + input + 
      ", state transition: " + currentState + 
      " to " + nextState
    );

    if(method != null){
      try {
        // only transition to the next state if the operation succeeds
        // TODO should we define failure behavior here? Maybe just logging
        if((Boolean)method.invoke(this) && nextState != null){
          setCurrentState(nextState);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    } else if(nextState != null) {
      // no operation, just transition straight to the next state
      setCurrentState(nextState);
    }
  }

  private Object[] lookupOperationAndNextState(State currentState, Input currentInput) {
    // cycle through the state transitions looking for a match for this state/input combination
    // there should only ever be one unique combination of state and input
    if(currentState != null && currentInput != null){
        for(Object[] transition : stateTransitionTable){
            State state = (State) transition[0];
            Input input = (Input) transition[1];
            String operation = (String)transition[2];
            State next = (State) transition[3];
            if(state == currentState && input == currentInput){
                return new Object[]{operation, next};
            }
        }
    }
    // TODO should probably log an error here
    return null;
  }
}

