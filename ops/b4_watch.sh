#!/usr/bin/env bash
# Wait for both teach_perception runs, then queue and start the B4 arms (closed-loop with each
# library). Operator infrastructure.
T=$HOME/workspace/ccd_exp/sessions/teach
until [ -f $T/teacher_on/test_score.json ] && [ -f $T/teacher_off/test_score.json ]; do sleep 120; done
cd $HOME/workspace/ccd_exp
J=sessions/ladder/jobs_b4.txt; : > $J
for a in a1 a2 a3; do for r in 25378 3255 25381 26405 3514 25424 27494 2091 17569 28198; do
  echo "ladder/B4_sensors_code_perc_teacherlib/$a/route_$r $r sensors claude-opus-5 CODE_TOOL=on PERCEPTION_ADDR=127.0.0.1:8795 CODE_LIB=$T/teacher_on/final_lib" >> $J
  echo "ladder/B4c_sensors_code_perc_selflib/$a/route_$r $r sensors claude-opus-5 CODE_TOOL=on PERCEPTION_ADDR=127.0.0.1:8795 CODE_LIB=$T/teacher_off/final_lib" >> $J
done; done
echo "$(date -Is) queued $(wc -l < $J) B4 jobs"
bash -c 'port_busy() { (exec 3<>/dev/tcp/127.0.0.1/$1) 2>/dev/null; }; until ! port_busy 20010 && ! port_busy 8791 && ! port_busy 20020 && ! port_busy 8792; do sleep 60; done'
echo "$(date -Is) slots 1-2 free: starting B4 lanes"
exec bash $HOME/workspace/lanes3.sh $HOME/workspace/ccd_exp $HOME/workspace/ccd_exp/$J 2 1
