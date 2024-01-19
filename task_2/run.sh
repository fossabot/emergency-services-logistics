echo "Sto compilando le classi Java"

domain_file="pddl_instances/esl-domain.pddl"
problem_file=""
selected_problem=$1

if [ "$selected_problem" -eq 1 ]; then
  problem_file="pddl_instances/esl-p1.pddl"
elif [ "$selected_problem" -eq 2 ]; then
  problem_file="pddl_instances/esl-p2.pddl"
elif [ "$selected_problem" -eq 3 ]; then
  problem_file="pddl_instances/esl-p3.pddl"
else
	echo "devi inserire un numero tra 1-3 per selezionare la istanza da risolvere"
	exit 1
fi

javac -d classes -cp ./lib/pddl4j-4.0.0.jar ./src/esl/*.java ./src/esl/heuristic/*.java ./src/esl/search/*.java ./src/utility/*.java

echo "Classi compilate"
echo "Esecuzione codice..."



java -cp classes:lib/pddl4j-4.0.0.jar esl.EslPlanner $domain_file $problem_file -e AJUSTED_SUM -l INFO
