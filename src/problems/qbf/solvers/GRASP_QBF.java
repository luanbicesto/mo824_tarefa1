package problems.qbf.solvers;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

import metaheuristics.grasp.AbstractGRASP;
import problems.qbf.QBF_Inverse;
import solutions.Solution;



/**
 * Metaheuristic GRASP (Greedy Randomized Adaptive Search Procedure) for
 * obtaining an optimal solution to a QBF (Quadractive Binary Function --
 * {@link #QuadracticBinaryFunction}). Since by default this GRASP considers
 * minimization problems, an inverse QBF function is adopted.
 * 
 * @author ccavellucci, fusberti
 */
public class GRASP_QBF extends AbstractGRASP<Integer> {

    private static final int REPAIR_FREQUENCY_SIZE = 4;
    private static final int ADD_CL_FREQUENCY_SIZE = 8;
    private static final int ADD_CL_CONSTANT_SIZE = 5;
    
	/**
	 * Constructor for the GRASP_QBF class. An inverse QBF objective function is
	 * passed as argument for the superclass constructor.
	 * 
	 * @param alpha
	 *            The GRASP greediness-randomness parameter (within the range
	 *            [0,1])
	 * @param iterations
	 *            The number of iterations which the GRASP will be executed.
	 * @param filename
	 *            Name of the file for which the objective function parameters
	 *            should be read.
	 * @throws IOException
	 *             necessary for I/O operations.
	 */
	public GRASP_QBF(Double alpha, Integer iterations, String filename) throws IOException {
		super(new QBF_Inverse(filename), alpha, iterations);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see grasp.abstracts.AbstractGRASP#makeCL()
	 */
	@Override
	public ArrayList<Integer> makeCL() {

		ArrayList<Integer> _CL = new ArrayList<Integer>();
		for (int i = 0; i < ObjFunction.getDomainSize(); i++) {
			Integer cand = new Integer(i);
			_CL.add(cand);
		}

		return _CL;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see grasp.abstracts.AbstractGRASP#makeRCL()
	 */
	@Override
	public ArrayList<Integer> makeRCL() {

		ArrayList<Integer> _RCL = new ArrayList<Integer>();

		return _RCL;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see grasp.abstracts.AbstractGRASP#updateCL()
	 */
	@Override
	public void updateCL() {

		// do nothing since all elements off the solution are viable candidates.

	}

	/**
	 * {@inheritDoc}
	 * 
	 * This createEmptySol instantiates an empty solution and it attributes a
	 * zero cost, since it is known that a QBF solution with all variables set
	 * to zero has also zero cost.
	 */
	@Override
	public Solution<Integer> createEmptySol() {
		Solution<Integer> sol = new Solution<Integer>();
		sol.cost = 0.0;
		return sol;
	}

	/**
	 * {@inheritDoc}
	 * 
	 * The local search operator developed for the QBF objective function is
	 * composed by the neighborhood moves Insertion, Removal and 2-Exchange.
	 */
	@Override
	public Solution<Integer> localSearch() {

	    int applyRepairCount = 0;
	    int applyAddClCount = 0;
		Double minDeltaCost;
		Integer candInTrash = null;
		Integer bestCandIn = null, bestCandOut = null;
		int repairFrequency = rng.nextInt(REPAIR_FREQUENCY_SIZE) + 1;
		int addToClFrequency = getAddToClFrequency();
		CLRemovedVariables = new ArrayList<Integer>();

		do {
			minDeltaCost = Double.POSITIVE_INFINITY;
			updateCL();
			applyRepairCount++;
			applyAddClCount++;
				
			// Evaluate insertions
			for (Integer candIn : CL) {
				double deltaCost = ObjFunction.evaluateInsertionCost(candIn, incumbentSol);
				if (deltaCost < minDeltaCost) {
					minDeltaCost = deltaCost;
					bestCandIn = candIn;
					bestCandOut = null;
				}
			}
			
			if(applyAddClCount == addToClFrequency) {
			    applyAddClCount = 0;
			    addToClFrequency = Double.compare(rng.nextDouble(), 0.1) <= 0 ? getAddToClFrequency() : ADD_CL_CONSTANT_SIZE;
			    // Evaluate insertions from trash list
			   
	            for (Integer candIn : CLRemovedVariables) {
	                double deltaCost = ObjFunction.evaluateInsertionCost(candIn, incumbentSol);
	                if (deltaCost < minDeltaCost) {
	                    minDeltaCost = deltaCost;
	                    bestCandIn = candIn;
	                    bestCandOut = null;
	                    candInTrash = candIn;
	                }
	            }
			}
			
			// Evaluate removals
			for (Integer candOut : incumbentSol) {
				double deltaCost = ObjFunction.evaluateRemovalCost(candOut, incumbentSol);
				if (deltaCost < minDeltaCost) {
					minDeltaCost = deltaCost;
					bestCandIn = null;
					bestCandOut = candOut;
				}
			}
			// Evaluate exchanges
			for (Integer candIn : CL) {
				for (Integer candOut : incumbentSol) {
					double deltaCost = ObjFunction.evaluateExchangeCost(candIn, candOut, incumbentSol);
					if (deltaCost < minDeltaCost) {
						minDeltaCost = deltaCost;
						bestCandIn = candIn;
						bestCandOut = candOut;
					}
				}
			}
			// Implement the best move, if it reduces the solution cost.
			if (minDeltaCost < -Double.MIN_VALUE) {
				if (bestCandOut != null) {
					incumbentSol.remove(bestCandOut);
					CL.add(bestCandOut);
				}
				if (bestCandIn != null) {
				    if(bestCandIn == candInTrash) {
				        CLRemovedVariables.remove(candInTrash);
				    }
					incumbentSol.add(bestCandIn);
					CL.remove(bestCandIn);
					count += 1;
				}
				if(applyRepairCount == repairFrequency) {
				    repair();
				    applyRepairCount = 0;
				    repairFrequency = rng.nextInt(REPAIR_FREQUENCY_SIZE) + 1;
				}
				
				ObjFunction.evaluate(incumbentSol);
			}
		} while (minDeltaCost < -Double.MIN_VALUE);

		repair();
		return null;
	}
	
	@Override
	public Solution<Integer> localSearchFirst() {

	    int applyRepairCount = 0;
	    int applyAddClCount = 0;
		Double minDeltaCost;
		Integer candInTrash = null;
		Integer bestCandIn = null, bestCandOut = null;
		int repairFrequency = rng.nextInt(REPAIR_FREQUENCY_SIZE) + 1;
		int addToClFrequency = getAddToClFrequency();
		CLRemovedVariables = new ArrayList<Integer>();
		boolean finded;

		do {
			finded = false;
			minDeltaCost = Double.POSITIVE_INFINITY;
			updateCL();
			applyRepairCount++;
			applyAddClCount++;
				
			// Evaluate insertions
			for (Integer candIn : CL) {
				double deltaCost = ObjFunction.evaluateInsertionCost(candIn, incumbentSol);
				if (deltaCost < minDeltaCost) {
					minDeltaCost = deltaCost;
					bestCandIn = candIn;
					bestCandOut = null;
					finded = true;
					break;
				}
			}
			
			// Evaluate removals
			if(!finded) {
				for (Integer candOut : incumbentSol) {
					double deltaCost = ObjFunction.evaluateRemovalCost(candOut, incumbentSol);
					if (deltaCost < minDeltaCost) {
						minDeltaCost = deltaCost;
						bestCandIn = null;
						bestCandOut = candOut;
						finded = true;
						break;
					}
				}
			}
			// Evaluate exchanges
			if(!finded) {
				for (Integer candIn : CL) {
					for (Integer candOut : incumbentSol) {
						double deltaCost = ObjFunction.evaluateExchangeCost(candIn, candOut, incumbentSol);
						if (deltaCost < minDeltaCost) {
							minDeltaCost = deltaCost;
							bestCandIn = candIn;
							bestCandOut = candOut;
							finded = true;
							break;
						}
					}
				}
			}
			// Implement the best move, if it reduces the solution cost.
			if (minDeltaCost < -Double.MIN_VALUE) {
				if (bestCandOut != null) {
					incumbentSol.remove(bestCandOut);
					CL.add(bestCandOut);
				}
				if (bestCandIn != null) {
				    if(bestCandIn == candInTrash) {
				        CLRemovedVariables.remove(candInTrash);
				    }
					incumbentSol.add(bestCandIn);
					CL.remove(bestCandIn);
					count += 1;
				}
				if(applyRepairCount == repairFrequency) {
				    repair();
				    applyRepairCount = 0;
				    repairFrequency = rng.nextInt(REPAIR_FREQUENCY_SIZE) + 1;
				}
				
				ObjFunction.evaluate(incumbentSol);
			}
		} while (minDeltaCost < -Double.MIN_VALUE);

		repair();
		return null;
	}
	
	private int getAddToClFrequency() {
	    return rng.nextInt(ADD_CL_FREQUENCY_SIZE - REPAIR_FREQUENCY_SIZE) + REPAIR_FREQUENCY_SIZE;
	}

 	public void repair() {
	    //simplestRepair();
	    //windowSizeTwo();
	    randomizedSimplestRepair();
	    ObjFunction.evaluate(incumbentSol);
	}
		
	public void windowSizeTwo() {
	    Double deltaCostCand1 = Double.POSITIVE_INFINITY;
	    Double deltaCostCand2 = Double.POSITIVE_INFINITY;
	    int removeCandIndex = -1;
	    Solution<Integer> incumbentSolCopy = new Solution<Integer>(incumbentSol);
        
	    sortSolution(incumbentSolCopy);
        
	    for(int index = 0; index < incumbentSolCopy.size(); index++) {
            if(index < (incumbentSolCopy.size() - 1) && applyAdjacentConstraint(incumbentSolCopy, index)) {
                deltaCostCand1 = ObjFunction.evaluateRemovalCost(incumbentSolCopy.get(index), incumbentSol);
                deltaCostCand2 = ObjFunction.evaluateRemovalCost(incumbentSolCopy.get(index+1), incumbentSol);
                
                removeCandIndex = deltaCostCand1 < deltaCostCand2 ? index+1 : index; 
                
                removeElementByValue(incumbentSol, incumbentSolCopy.get(removeCandIndex));
                incumbentSolCopy.remove(removeCandIndex);
            }
        }
	}
	
	public void randomizedSimplestRepair() {
	    double removeCandIndexProb = 0;
	    int removeCandIndex = 0;
        Solution<Integer> incumbentSolCopy = new Solution<Integer>(incumbentSol);
        sortSolution(incumbentSolCopy);
        
        /*Simplest repair: remove the right element that is incorrect*/
        for(int index = 0; index < incumbentSolCopy.size(); index++) {
            if(index < (incumbentSolCopy.size() - 1) && applyAdjacentConstraint(incumbentSolCopy, index)) {
                removeCandIndexProb = rng.nextDouble();
                removeCandIndex = Double.compare(removeCandIndexProb, 0.5) <= 0 ? index : index + 1;
                
                CLRemovedVariables.add(incumbentSolCopy.get(removeCandIndex));
                
                removeElementByValue(incumbentSol, incumbentSolCopy.get(removeCandIndex));
                incumbentSolCopy.remove(removeCandIndex);
            }
        }
    }
	
	public void simplestRepair() {
	    Solution<Integer> incumbentSolCopy = new Solution<Integer>(incumbentSol);
	    sortSolution(incumbentSolCopy);
	    
	    /*Simplest repair: remove the right element that is incorrect*/
	    for(int index = 0; index < incumbentSolCopy.size(); index++) {
	        if(index < (incumbentSolCopy.size() - 1) && applyAdjacentConstraint(incumbentSolCopy, index)) {
	            removeElementByValue(incumbentSol, incumbentSolCopy.get(index + 1));
	            incumbentSolCopy.remove(index + 1);
	        }
	    }
	}
	
	private void sortSolution(Solution<Integer> sol) {
        sol.sort(new Comparator<Integer>() {
            @Override
            public int compare(Integer element1, Integer element2)
            {

                return  element1.compareTo(element2);
            }
        });
    }

	public void removeElementByValue(Solution<Integer> solution, int targetValue) {
	    for(int index = 0; index < solution.size(); index++) {
	        if(solution.get(index).intValue() == targetValue) {
	            solution.remove(index);
	            break;
	        }
	    }
	}
	
	private boolean applyAdjacentConstraint(Solution<Integer> currentSolution, int currentIndex) {
	    return currentSolution.get(currentIndex) + 1 == currentSolution.get(currentIndex+1);
	}
	
	/**
	 * A main method used for testing the GRASP metaheuristic.
	 * 
	 */
	public static void main(String[] args) throws IOException {

		long startTime = System.currentTimeMillis();
		GRASP_QBF grasp = new GRASP_QBF(0.00, 1000, "instances/qbf080");
		Solution<Integer> bestSol = grasp.solve();
		System.out.println("maxVal = " + bestSol);
		long endTime   = System.currentTimeMillis();
		long totalTime = endTime - startTime;
		System.out.println("Time = "+(double)totalTime/(double)1000+" seg");

	}

}
