

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Scanner;

import nju.lzx.Constraint.*;
import nju.lzx.Data.*;
import nju.lzx.Algorithm.*;
import nju.lzx.Interface.*;
import nju.lzx.LocalSearchOperator.*;
import nju.lzx.InitialSolutionOperator.*;
import nju.lzx.VehicleReductionOperator.*;
import nju.lzx.Utility.*;
import nju.lzx.Route.*;


// TODO: Auto-generated Javadoc
/**
 * Vehicle Routing Problem with Time Windows (VRPTW)���ʾ����.
 */
public class VRPTW {

	
	/**
	 * ��������.
	 *
	 * @param args ������
	 * @throws IOException IO�쳣��
	 */
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		//System.setOut(new PrintStream(new FileOutputStream("log4.txt")));
		double t1 = System.nanoTime();
		Instance inst = load_instance("data/homberger_1000/c1_10_2.txt", 1000);
		inst.m = 90;
		//����ģʽ����
		inst.parameter.Mode.multi_thread_enable = true;
		System.setProperty("java.util.concurrent.ForkJoinPool.common.parallelism", "8");
		//���ó�ʼ�����
		inst.parameter.InitialSolution.log_print = false;
		//���ý�����������
		inst.parameter.TabuSearch.maximum_iteration = 50;
		inst.parameter.TabuSearch.maximum_tabu_tenure = 100;
		inst.parameter.TabuSearch.mininum_shake_tenure = 100;
		inst.parameter.TabuSearch.minimum_shake_iteration = 50;
		inst.parameter.TabuSearch.log_print = false;
		inst.parameter.TabuSearch.log_detail = false;
		//�������Ӳ���
		inst.parameter.Operator.insertion_prune_threshhold = 1.0;
		inst.parameter.Operator.exchange_prune_threshhold = 1.0;
		inst.parameter.Operator.cross_prune_threshhold = 1.0;
		inst.parameter.Operator.remove_prune_threshhold = 0.0;
		inst.parameter.Operator.route_cross_threshhold = 1.5;
		//���ü��ٳ����㷨����
		inst.parameter.VehicleReduction.insert_search_enable = true;
		inst.parameter.VehicleReduction.cost_maximum_iteration = 100;
		inst.parameter.VehicleReduction.outer_maximum_iteration = 100;
		inst.parameter.VehicleReduction.random_remove_ratio = 0.05;
		inst.parameter.VehicleReduction.log_print = true;
		
		System.out.println("Multiple Thread��" + inst.parameter.Mode.multi_thread_enable);
		
		//����Լ������
		Constraint[] cnts = new Constraint[3];
		MinimizeDistance.ConstraintData dist_dat = new MinimizeDistance.ConstraintData(inst.d, 1);
		cnts[0] = new MinimizeDistance(dist_dat, 1);
		CapacityConstraint.ConstraintData cap_dat = new CapacityConstraint.ConstraintData(inst.q, inst.Q);
		cnts[1] = new CapacityConstraint(cap_dat, true, 100);
		TimeWindowConstraint.ConstraintData tw_dat = new TimeWindowConstraint.ConstraintData(inst.e, inst.l, inst.t, inst.s);
		cnts[2] = new TimeWindowConstraint(tw_dat, true, 100);
		
		//�����㷨����
		Operator[] operators = new Operator[4];
		double[] coefs = new double[4];
		operators[0] = new RelocateBase(inst);
		coefs[0] = 1;
		operators[1] = new ExchangeBaseDeep(inst);
		coefs[1] = 0.5;
		operators[2] = new CrossBase(inst);
		coefs[2] = 1;
		operators[3] = new RelocateBaseIntra(inst);
		coefs[3] = 1;
		
		//������Ҫ���ʵĽڵ㼯��
		ArrayList<Atr> atrs = new ArrayList<Atr>();
		for(int i = 1; i < inst.n; i++){
			atrs.add(new Atr(i));
		}
		boolean[] exc = new boolean[inst.n];
		for(int i = 1; i < inst.n; i++)
			exc[i] = true;
		
		//�����ʼ��
		Greedy greedy = new Greedy(inst, new InsertBase(inst, cnts));
		ArrayList<Route> s = greedy.generate(atrs);
		System.out.println(greedy.toString(s));
		greedy.check(s, true, exc);
		System.out.println("feasibility of the initial solution>>>" + greedy.is_feasible(s) + "\t" + s.size() + "\t" + greedy.get_total_cost(s));
		//System.exit(0);
		
		//��С��������Ŀ
		TabuSearch tabu = new TabuSearch(inst, operators, coefs);
		VehicleMinimizeBase vmb = new VehicleMinimizeBase(inst);
		VehicleReductionAlgo vra = new VehicleReductionAlgo(inst, tabu, vmb, inst.m, exc);
		s = toDeep(inst, s);
		vra.check(s, true, exc);
		ArrayList<Route> bs = vra.solve(s);

		vra.check(bs, true, exc);
		System.out.println(vra.toString(bs));
		double t2 = System.nanoTime();
		System.out.println("computation time>>>" + (t2 - t1) / 1e9);
		inst.statistics.print();
		write_solution("result/c1_10_8.txt", bs);
	}
	
	/**
	 * ����������.
	 *
	 * @param path ����·����
	 * @param _n �˿ͽڵ�������
	 * @return ����������
	 * @throws FileNotFoundException IO�쳣��
	 */
	public static Instance load_instance(String path, int _n) throws FileNotFoundException{
		Instance inst = new Instance();
		Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));
		inst.name = cin.next();
		for(int i = 0; i < 3; i++){
			cin.next();
		}
		inst.m = cin.nextInt();
		inst.Q = cin.nextDouble();
		for(int i = 0; i < 12; i++){
			cin.next();
		}
		inst.n = _n + 1;
		inst.s = new double[inst.n];
		inst.e = new double[inst.n];
		inst.l = new double[inst.n];
		inst.q = new double[inst.n];
		inst.lng = new double[inst.n];
		inst.lat = new double[inst.n];
		for(int i = 0; i < inst.n; i++){
			cin.next();
			inst.lng[i] = cin.nextDouble();
			inst.lat[i] = cin.nextDouble();
			inst.q[i] = cin.nextDouble();
			inst.e[i] = cin.nextDouble();
			inst.l[i] = cin.nextDouble();
			inst.s[i] = cin.nextDouble();
		}
		cin.close();
		inst.d = new double[inst.n][inst.n];
		inst.t = new double[inst.n][inst.n];
		for(int i = 0; i < inst.n; i++){
			for(int j = i + 1; j < inst.n; j++){
				inst.d[i][j] = inst.d[j][i] = inst.t[i][j] = inst.t[j][i] = 
						Math.sqrt((inst.lng[i] - inst.lng[j]) * (inst.lng[i] - inst.lng[j]) + (inst.lat[i] - inst.lat[j]) * (inst.lat[i] - inst.lat[j]));
			}
		}
		return inst;
	}
	
	/**
	 * ��һ����ͨ�Ľ�ת����һ�����Ա�ExchangeBaseDeep��RelocateBseIntra�����Ľ⣬��·���а�����·����.
	 *
	 * @param inst ������Ϣ��
	 * @param s ��ǰ�⡣
	 * @return �����µĽ⡣
	 */
	public static ArrayList<Route> toDeep(Instance inst, ArrayList<Route> s){
		ArrayList<Route> ns = new ArrayList<Route>();
		for(int i = 0; i < s.size(); i++){
			Route r = s.get(i);
			Reference ref = r.get_reference();
			Route nr = new RouteBase(inst, ref.len, ref.seq, false, true);
			Constraint[] _cnts = r.get_constraints();
			Constraint[] _cnts2 = new Constraint[_cnts.length];
			for(int j = 0; j < _cnts.length; j++){
				_cnts2[j] = _cnts[j].copy(nr.get_reference());
			}
			nr.add_constraints(_cnts2);
			ns.add(nr);
		}
		return ns;
	}
	
	
	/**
	 * ���Ᵽ�浽�ļ���
	 *
	 * @param file �ļ����ơ�
	 * @param s ��ǰ�⡣
	 * @throws IOException I/O�쳣��
	 */
	public static void write_solution(String file, ArrayList<Route> s) throws IOException{
		BufferedWriter out = new BufferedWriter(new FileWriter(file));
		double total_cost = 0;
		for(int i = 0; i < s.size(); i++){
			total_cost += s.get(i).get_total_cost();
		}
		out.write(s.size() + " " + total_cost);
		out.newLine();
		for(int i = 0; i < s.size(); i++){
			Route r = s.get(i);
			for(int j = 0; j < r.get_size(); j++){
				int id = r.get_node(j);
				out.write(id + " ");
			}
			out.newLine();
		}
		out.close();
	}

}
