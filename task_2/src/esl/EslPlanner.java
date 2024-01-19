/*
 * Copyright (c) 2010 by Damien Pellier <Damien.Pellier@imag.fr>.
 *
 * This file is part of PDDL4J library.
 *
 * PDDL4J is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PDDL4J is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PDDL4J.  If not, see <http://www.gnu.org/licenses/>
 */

package esl;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.SearchStrategy;

import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

import java.util.Arrays;

/**
 * This class implements Fast Forward planner based on Enforced Hill Climbing algorithm and AStar search.
 *
 *
 * <p>The command line syntax to launch the planner is as follow:</p>
 *
 * <pre>
 * {@code
 * EslPlanner [-hV] [-l=<logLevel>] [-t=<timeout>] [-w=<weight>] <domain>
 *             <problem>
 *
 * Description:
 *
 * Solves a specified planning problem combining enforced hill climbing and A*
 * search strategies using the the delete relaxation heuristic.
 *
 * Parameters:
 *       <domain>              The domain file.
 *       <problem>             The problem file.
 *
 * Options:
 *   -l, --log=<logLevel>      Set the level of trace: ALL, DEBUG, INFO, ERROR,
 *                               FATAL, OFF, TRACE (preset INFO).
 *   -t, --timeout=<timeout>   Set the time out of the planner in seconds (
 *                               preset 600s).
 *   -w, --weight=<weight>     Set the weight of the heuristic (preset 1.0).
 *   -h, --help                Show this help message and exit.
 *   -V, --version             Print version information and exit.
 *  }
 * </pre>
 *
 * <p>Command line example:</p>
 * <pre>
 * {@code
 *    java -cp build/libs/pddl4j-4.0-all.jar esl.EslPlanner
 *            src/test/resources/benchmarks/pddl/ipc2002/depots/strips-automatic/domain.pdd
 *            src/test/resources/benchmarks/pddl/ipc2002/depots/strips-automatic/p01.pddl
 *            -t 1000
 * }
 * </pre>
 *
 * @author Carmelo Gugliotta
 * @author Vittorio Faiella
 * @author Carmelo Pecora
 * @version 2.0 - 24.01.2018
 *
 */
@CommandLine.Command(name = "EslPlanner",
    version = "EslPlanner 2.0",
    description = "Solves a specified planning problem combining enforced hill climbing and A* search "
        + "strategies using the the delete relaxation heuristic.",
    sortOptions = false,
    mixinStandardHelpOptions = true,
    headerHeading = "Usage:%n",
    synopsisHeading = "%n",
    descriptionHeading = "%nDescription:%n%n",
    parameterListHeading = "%nParameters:%n",
    optionListHeading = "%nOptions:%n")
public final class EslPlanner extends AbstractStateSpacePlanner  {

    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(EslPlanner.class.getName());

    /**
     * Creates a new planner with default parameters.
     */
    public EslPlanner() {
        this(EslPlanner.getDefaultConfiguration());
    }

    /**
     * Creates a new planner.
     *
     * @param configuration the configuration of the planner.
     */
    public EslPlanner(PlannerConfiguration configuration) {
        super(configuration);
    }

    /**
     * This method return the default arguments of the planner.
     *
     * @return the default arguments of the planner.
     * @see PlannerConfiguration
     */
    public static PlannerConfiguration getDefaultConfiguration() {
        PlannerConfiguration config = Planner.getDefaultConfiguration();
        config.setProperty(EslPlanner.SEARCH_STRATEGIES_SETTING,Arrays.asList(

                SearchStrategy.Name.ASTAR,
                SearchStrategy.Name.ENFORCED_HILL_CLIMBING
                )
            );
        return config;
    }

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = { "-w", "--weight" }, defaultValue = "1.0", paramLabel = "<weight>",
        description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        super.setHeuristicWeight(weight);
    }

    /**
     * Set the name of heuristic used by the planner to solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = { "-e", "--heuristic" }, defaultValue = "FAST_FORWARD",
            description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, MAX, FAST_FORWARD, "
                    + "SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public final void setHeuristic(StateHeuristic.Name heuristic)  {
        super.setHeuristic(heuristic);
    }

    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be instantiated.
     */
    @Override
    public Problem instantiate(final DefaultParsedProblem problem) {
        Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
     * Checks the planner configuration and returns if the configuration is valid. A configuration is valid if (1) the
     * domain and the problem files exist and can be read, (2) the timeout is greater than 0, (3) the weight of the
     * heuristic is greater than 0, (4) the heuristic is a not null and (5) the list of search strategies to use is
     * ASTAR and ENFORCE_HILL_CLIMBING.
     *
     * @return <code>true</code> if the configuration is valid <code>false</code> otherwise.
     */
    public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
            && this.getSearchStrategies().size() == 2
            && this.getSearchStrategies().contains(SearchStrategy.Name.ENFORCED_HILL_CLIMBING)
            && this.getSearchStrategies().contains(SearchStrategy.Name.ASTAR);
    }

    /**
     * The main method of the <code>FF</code> planner.
     *
     * @param args the arguments of the command line.
     */
    public static void main(String[] args) {
        try {
            final EslPlanner planner = new EslPlanner();
            CommandLine cmd = new CommandLine(planner);
            int exitCode = (int) cmd.execute(args);
            if (exitCode == 1) {
                LOGGER.fatal(cmd.getUsageMessage());
            }
            System.exit(exitCode);
        } catch (Throwable e) {
            LOGGER.fatal(e.getMessage());
        }
    }
}
