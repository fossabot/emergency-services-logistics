package esl.heuristic;

import java.util.*;

public class Predicate {


    // Predicate ID
    private final int symbol;
    // Predicate name
    private final String name;//Nome_predicato


    // List of Arguments involved in the predicate: an argument has an ID and his name
    private final List<Argument> arguments;

    public Predicate(int symbol, String name, int[] arguments_id, String[] arguments_name) {
        this.symbol = symbol;
        this.name = name;
        this.arguments = new LinkedList<>();
        for (int i = 0; i < arguments_id.length; i++) {
            int argument_id = arguments_id[i];
            String argument_name = arguments_name[i + 1];
            arguments.add(new Argument(argument_id, argument_name));
        }
    }

    public Predicate(String name,List<Argument> arguments){
        this.symbol=-1;
        this.name=name;
        this.arguments=new LinkedList<>();
        for(Argument arg:arguments){
            arguments.add(arg.clone());
        }
    }

    public Predicate(String name,Argument argument){
        this.symbol=-1;
        this.name=name;
        this.arguments=new LinkedList<>();
        this.arguments.add(argument);

    }


    public boolean containsArgByID(int id){

        for(Argument arg:arguments){
            int curr_id=arg.getArgument_id();
            if(id==curr_id) return true;
        }

        return false;
    }

    public boolean containsArgByName(String name){

        for(Argument arg:arguments){
            String curr_name=arg.getArgument_name();
            if(name.equals(curr_name)) return true;
        }

        return false;
    }

    public int getSymbol() {
        return symbol;
    }

    public String getName() {
        return name;
    }



    public List<Argument> getArguments() {
        List<Argument> clonedArguments = new LinkedList<>();

        for (Argument argument : this.arguments) {
            Argument clonedArgument =argument.clone();
            clonedArguments.add(clonedArgument);
        }

        return clonedArguments;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Predicate predicate = (Predicate) o;
        return (symbol == predicate.symbol || name.equals(predicate.name)) && //Il nome e il simbolo di un predicato sono unici.
                arguments.equals(predicate.arguments);
    }

    @Override
    public int hashCode() {
        return Objects.hash(symbol, name, arguments);
    }

    @Override
    public String toString() {

        StringBuilder result = new StringBuilder("(")
                .append("<").append(symbol).append(":").append(name).append(">");

        for (Argument arg : arguments) {
            result.append(" ").append("<").append(arg.getArgument_id()).append(":").append(arg.getArgument_name()).append(">");
        }

        result.append(")");


        return result.toString();
    }
}




