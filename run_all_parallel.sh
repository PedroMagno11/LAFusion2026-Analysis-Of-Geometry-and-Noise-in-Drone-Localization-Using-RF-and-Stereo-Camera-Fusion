#!/bin/bash
# Roda todas as combinações (cenário, repetição) em paralelo, usando xargs -P.
#
# Uso: ./run_all_parallel.sh <n_repeticoes> <n_processos_paralelos>
# Exemplo: ./run_all_parallel.sh 30 4   (30 repetições, 4 processos simultâneos)

N_REPS=${1:-30}
N_PARALLEL=${2:-4}

echo "Rodando 3 cenários x ${N_REPS} repetições, com ${N_PARALLEL} processos em paralelo..."
echo "Isso ainda pode levar bastante tempo (o EKF local+global é intensivo em CPU)."

combos_file=$(mktemp)
for scenario in 1 2 3; do
    for rep in $(seq 0 $((N_REPS - 1))); do
        echo "$scenario $rep" >> "$combos_file"
    done
done

echo "Total de combinações: $(wc -l < "$combos_file")"

cat "$combos_file" | xargs -P "$N_PARALLEL" -L 1 sh -c \
    'python3 run_single_repetition.py "$0" "$1"'

rm -f "$combos_file"

echo ""
echo "Todas as repetições concluídas. Agregando resultados..."

# CORREÇÃO: antes, este bloco reimplementava a agregação inline (com
# aggregate_runs/rmse_m, que ficaram desatualizados assim que
# metrics_evaluator.py mudou de esquema - exatamente o tipo de
# duplicação que causa esse drift). Agora reusa a mesma
# aggregate_and_report() de run_all_parallel.py, que já sabe juntar os
# arquivos de métrica (scenario*_rep*.csv) E os de correlação GDOP
# (gdop_scenario*_rep*.csv, item 4) gerados por run_single_repetition.py.
python3 -c "from run_all_parallel import aggregate_and_report; aggregate_and_report()"
