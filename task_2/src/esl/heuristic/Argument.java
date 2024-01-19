package esl.heuristic;

import java.util.Objects;

public class Argument implements Cloneable {
    private int argument_id;
    private String argument_name;

    public Argument(int argument_id, String argument_name) {
        this.argument_id = argument_id;
        this.argument_name = argument_name;
    }

    public int getArgument_id() {
        return argument_id;
    }



    public String getArgument_name() {
        return argument_name;
    }

    @Override
    public String toString() {
        return "<" + argument_id +
                "," + argument_name + '>';
    }

    @Override
    public Argument clone() {
        try {
            Argument clone = (Argument) super.clone();
            clone.argument_id=this.argument_id;
            clone.argument_name=this.argument_name;
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Argument argument = (Argument) o;
        return argument_id == argument.argument_id || argument_name.equals(argument.argument_name); //Sia l'id che il nome di un argomento sono unici

    }

    @Override
    public int hashCode() {
        return Objects.hash(argument_id, argument_name);
    }
}
