package utility;

import esl.heuristic.Argument;
import esl.heuristic.Predicate;

import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class Utility {


    public static List<Predicate> getGoalAlreadySatisfied(List<Predicate> predicates,List<Predicate> goals){
        return predicates.stream()
                .filter(goals::contains)
                .collect(Collectors.toList());
    }

    public static List<Predicate> getGoalNotAlreadySatisfied(List<Predicate> goals, List<Predicate> goalsAlreadySatisfied){
        return goals.stream()
                .filter(goal -> !goalsAlreadySatisfied.contains(goal))
                .collect(Collectors.toList());
    }


    public static List<Argument> getListOfElementsRequiredToAchiveTheGoal(List<Predicate> goalsNotSatisfiedYet){
        return  goalsNotSatisfiedYet.stream()
                .flatMap(p -> {
                    if ("satisfied-with-at-least-one".equals(p.getName())) {
                        // If the predicate is "satisfied-with-at-least-one," there are two objects
                        return Stream.of(
                                p.getArguments().get(1).clone(),
                                p.getArguments().get(2).clone()
                        );
                    } else {
                        // Otherwise, return only the second argument
                        return Stream.of(p.getArguments().get(1).clone());
                    }
                })
                .collect(Collectors.toList());
    }




}
