function set_prefix(prefix)

vars = struct;
vars.a2  = 'pm';
vars.a3  = 'Params';
vars.a4  = 'Vars';
vars.a5  = 'Workspace';
vars.a6  = 'Settings';
vars.a7  = 'ldl_solve(';
vars.a8  = 'ldl_factor(';
vars.a9  = 'check_factorization(';
vars.a10 = 'matrix_multiply(';
vars.a11 = 'check_residual(';
vars.a12 = 'fill_KKT(';
vars.a13 = 'multbymA(';
vars.a14 = 'multbymAT(';
vars.a15 = 'multbymG(';
vars.a16 = 'multbymGT(';
vars.a17 = 'multbyP(';
vars.a18 = 'fillq(';
vars.a19 = 'fillh(';
vars.a20 = 'fillb(';
vars.a21 = 'pre_ops(';
vars.a22 = 'eval_gap(';
vars.a23 = 'set_defaults(';
vars.a24 = 'setup_pointers(';
vars.a25 = 'setup_indexing(';
vars.a26 = 'set_start(';
vars.a27 = 'eval_objv(';
vars.a28 = 'fillrhs_aff(';
vars.a29 = 'fillrhs_cc(';
vars.a30 = 'refine(';
vars.a31 = 'calc_ineq_resid_squared(';
vars.a32 = 'calc_eq_resid_squared(';
vars.a33 = 'better_start(';
vars.a34 = 'fillrhs_start(';
vars.a35 = 'solve(';
vars.a36 = 'main(';
vars.a37 = 'load_default_data(';
vars.a38 = 'tic(';
vars.a39 = 'toc(';
vars.a40 = 'printmatrix(';
vars.a41 = 'unif(';
vars.a42 = 'ran1(';
vars.a43 = 'randn(';
vars.a44 = 'reset_rand(';
vars.a45 = 'params';
vars.a46 = 'vars';
vars.a47 = 'work';
vars.a48 = 'settings';
vars.a49 = 'tocq(';
vars.a50 = 'randn_internal(';
vars.a51 = 'SOLVER_H';
vars.a52 = 'solver.h';
vars.a53 = 'solver.o';
vars.a54 = 'testsolver';
vars.a55 = 'matrix_support.o';
vars.a56 = 'ldl.o';
vars.a57 = 'util.o';
vars.a58 = 'global_seed';

files = struct;
files.a1 = 'solver.h';
files.a2 = 'util.c';
files.a3 = 'ldl.c';
files.a4 = 'matrix_support.c';
files.a5 = 'solver.c';
files.a6 = 'testsolver.c';
files.a7 = 'description.cvxgen';

files.a8=  'Makefile';
files.a9 = 'csolve.m';
files.a10 = 'cvxsolve.m';
files.a11= 'make_csolve.m';
files.a12= 'csolve.c';

folder_path = 'cvxgen/';

mkdir([prefix folder_path]);

for i=1:7
    filename = getfield(files, sprintf('a%d',i));
    str = fileread([folder_path filename]);
    for j = 2:58
        word = getfield(vars, sprintf('a%d',j));
        str = strrep(str, word, [prefix word]);
    end
    if ~strcmp('Makefile',filename)
        filename = [prefix filename];
    end
    fid = fopen([prefix folder_path filename], 'w');
    fwrite(fid, str, '*char');              %# write characters (bytes)
    fclose(fid);
end
% for i=8:8
%     filename = getfield(files, sprintf('a%d',i));
%     str = fileread([folder_path filename]);
%     if strcmp('csolve.c',filename)
%         filename = [filename 'c'];
%     end
%     filename = [prefix filename];
%     fid = fopen([prefix folder_path filename], 'w');
%     fwrite(fid, str, '*char');              %# write characters (bytes)
%     fclose(fid);
% end