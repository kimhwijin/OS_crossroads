
#include <stdio.h>

#include "threads/thread.h"
#include "threads/interrupt.h"
#include "devices/timer.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"


/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
		/* to B */
		{{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
		/* to C */
		{{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
		/* to D */
		{{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}


//생성된 차량의 쓰레드개수를 저장하는 변수
static int thread_cnt = 0;
//static 변수에 Mutual exclution을 보장하기 위한 semaphore
static struct semaphore sem_TC;
//단위 스텝이 진행되는 동안 자신의 스텝이 종료되어 블록된 쓰레드의 개수
static int blocked_thread_cnt = 0;
//생성된 차량 쓰레드의 주소값을 저장하는 쓰레드 리스트
static struct thread* threadlist[50] = {NULL,};
//생성된 차량 쓰레드의 도착지점을 정수 변수로 나타낸 리스트 (threadlist와 동일한 순서)
static int destlist[50] = {-1,};
//단위 스텝에서 자신의 스텝이 종료되어 블록된 차량 쓰레드의 주소값을 저장하는 쓰레드 리스트
static struct thread* blocked_threadlist[50] = {NULL,};
//manager thread가 block되고 unblock 될때 사용되는 semaphore
static struct semaphore sem_TRI;
//현재 step이 deadlock이 발생되어서 회복하는 step인지 여부를 나타내는 변수
static bool is_deadlock_recover = false;
/* return 0:termination, 1:success, -1:fail */
int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			//전역변수를 접근 및 변경
			sema_down(&sem_TC);
				//현재 차량 쓰레드가 이번 스텝에 목적지에 도달함.
				struct thread *th = thread_current();
				bool findflag = false;
				//현재 thread를 threadlist에서 pop한다.
				for(int i = 0;i<thread_cnt;i++){
					//현재 thread 를 threadlist에서 찾아 threadlist에서 제거한다.
					if(th->tid == threadlist[i]->tid && !findflag){
						threadlist[i] = NULL;
						findflag = true;
						continue;
					}
					//나머지 쓰레드를 한칸 앞으로 옮겨 중간에 null값이 없도록한다.
					if(findflag){
						threadlist[i-1] = threadlist[i];
						destlist[i-1] = destlist[i];
					}
				}
				//전체 쓰레드 개수를 줄여줌.
				thread_cnt--;
				//종료 쓰레드가 마지막 쓰레드일 경우 가는길에 manager 쓰레드를 활성화 시켜주고 감
				if(thread_cnt == blocked_thread_cnt && !is_deadlock_recover)
					sema_up(&sem_TRI);

			sema_up(&sem_TC);//접근 종료

			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);

			return 0;//완료된 리턴값
		}
	}

	/* lock next position */

	//다음 위치의 lock 주소를 저장함
	struct lock *_lock = &vi->map_locks[pos_next.row][pos_next.col];
	//현재 차량 step이 움직임이 가능한지 불가능한지 여부 결과를 저장하는 변수
	bool breakflag = false;
	
	//다음 위치의 lock 획득시도를 반복하는 while문
	while(!lock_held_by_current_thread(_lock)){//while(!lock_try_acquire...)할경우에 assert !lock_held_by_current_thread에서 오류가 발생해 이런식으로 변경함
		//다음 위치 lock획득 시도
		if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])){
			//lock획득 성공시 while문 탈출 및 breakflag = false로 움직임 가능
			break;
		}
		//전역변수 접근
		sema_down(&sem_TC);
			
			*_lock = vi->map_locks[pos_next.row][pos_next.col];
			//lock holder가 없을경우(try_acquire시도 후 중간에 lock release 가 됐을 경우를 고려함)
			if(_lock->holder == NULL){
				//접근 종료 및 다시 lock 획득 시도
				sema_up(&sem_TC);
				continue;	
			}

			//이번 스텝에 blocked_threadlist를 검색해 다음칸 lock holder 와 비교
			for(int i = 0;i < blocked_thread_cnt; i++){
				//다음칸 lock hodler가 블럭된 쓰레드라면 이 쓰레드는 이번 스탭에 전진 불가능 판단
				if(blocked_threadlist[i]->tid == _lock->holder->tid){
					breakflag = true;
					break;
				}
			}
			//전진 불가능판단
			if (breakflag){
				//lock 획득 시도를 멈추고 접근 종료 및 전진 불가능한 차량으로 계속 진행
				sema_up(&sem_TC);
				break;
			}
		//접근 종료
		sema_up(&sem_TC);
		//while문에서 semaphore을 반복적으로 접근할시 다른 쓰레드의 접근이 어려울 수 있음을 고려해 시간텀을 두었음
		timer_msleep(10);
	}

	//움직임이 가능한 차량 쓰레드의 경우
	if (!breakflag){
		if (vi->state == VEHICLE_STATUS_READY) {
			/* start this vehicle */
			vi->state = VEHICLE_STATUS_RUNNING;
		} else {
			/* release current position */
			//데드락 회복 스텝이 아닐경우에만 현재 쓰레드를 release한다. 왜냐하면 데드락 회복시에는 교차로 내의 모든 lock이 풀려있기 때문에
			if(!is_deadlock_recover)
				lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		/* update position */
		vi->position = pos_next;
	}


	
	//전역번수 접근 및 자신의 step이 종료되고 blocked 상태로 변환 과정
	sema_down(&sem_TC);

		//현재 쓰레드를 blocked된 쓰레드에 저장한다
		blocked_threadlist[blocked_thread_cnt++] = thread_current();
		//모든 쓰레드가 블럭상태일 경우 manager 쓰레드를 풀어준다.
		printf("%d,%d//",blocked_thread_cnt,thread_cnt);
		if(blocked_thread_cnt == thread_cnt && !is_deadlock_recover){
			sema_up(&sem_TRI);
		//만일 데드락 회복 스텝일 경우 블럭된 쓰레드가 8 개일 경우 manager 쓰레드를 풀어준다.
		}else if(is_deadlock_recover && blocked_thread_cnt == 8){
			sema_up(&sem_TRI);
		}

	//접근 종료
	sema_up(&sem_TC);

	//현재 쓰레드를 블록하고, manager쓰레드의 unblock 을 기다린다. 자신의 스텝 종료
	enum intr_level old_level= intr_disable ();
	thread_block();
	intr_set_level (old_level);

	//unblock이 되었을때 움직임 여부에 따라서 성공 실패값을 리턴함
	if(!breakflag)
		return 1;
	else
		return -1;
}


//차량 스레드 시작시 수행 함수
void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';
	
	
	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	if(start == dest){
		vi->state = VEHICLE_STATUS_FINISHED;
		return;
	}
	//manager thread를 생성하기 위해서 처음 한개의 쓰레드를 찾기 위한 변수.
	bool fisrt_flag = false;
	//생성된 차량 쓰레드를 개수를 구하기 위해서 thread_cnt는 처음에 동기화가 진행되지 않은 상태이다.
	if (thread_cnt == 0){
		//따라서 여러개의 쓰레드가 thread_cnt == 0 임을 만족하고, 여러번의 초기화가 진행된다.(상관 없음)
		sema_init(&sem_TC,1);
		sema_init(&sem_TRI,0);
	}
	//crossroads에서 모든 쓰레드의 생성이 완료되고, 초기화가 완료될때까지 기다리는 시간. 
	//이게없으면 맨처음쓰레드가 sema_down했을때 나중에 생성된 쓰레드가 sema_init(1)을 통해 semaphore value가 이상해지는 상황이 발생함
	//이번 프로젝트에서 아쉬운부분
	timer_msleep(1000);

	//전역변수 접근 단1개의 쓰레드만 접근가능
	sema_down(&sem_TC);
		//맨처음 접근한 쓰레드의 경우 thread_cnt == 0 이다.
		if (thread_cnt == 0){
			//따라서 first_flag = true 로 변경해줌, thread_cnt 를 증가하기 때문에 다른 쓰레드가 first_flag = true 될 일은 존재하지않음.
			fisrt_flag = true;
		}
		struct thread *t = thread_current();
		//현재 쓰레드와 쓰레드의 목적지를 리스트에 저장함.(threadlist는 단위 스텝문제, destlist는 데드락 해결문제를 위해 사용한다.)
		threadlist[thread_cnt] = t;
		destlist[thread_cnt] = dest;

		//쓰레드개수와 , block된 쓰레드 개수도 증가시켜서 manager thread는 생성되고 block되는데 이를 해제시켜주기위해 사용함
		thread_cnt++;
		blocked_thread_cnt++;

	//접근 종료
	sema_up(&sem_TC);

	//처음 쓰레드의 경우 manager thread를 생성한다.
	if (fisrt_flag){
		//manager thread에 map_lock을 넘겨줘서 데드락 문제해결을 위해 사용한다.
		thread_create("unit step counter", PRI_DEFAULT, release_blocked_threads, vi->map_locks);
	}

	//생성된 쓰레드는 thread_cnt를 증가시키고 블록상태로 모든 쓰레드가 블록될 때까지 대기함.
	enum intr_level old_level= intr_disable ();
	thread_block();
	intr_set_level (old_level);
	

	//--
	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}

		/* termination condition. */ 
		if (res == 0) {
			if(thread_cnt == 0){
				sema_up(&sem_TRI);
			}
			break;
		}

		/* sleep for 1 sec */
		timer_msleep(1000);
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}

void release_blocked_threads(void **_maplock){

	//map_lock을 저장하기 위한 변수
	struct lock **maplock;
	maplock = (struct lock **)_maplock;
	//데드락 회복을 위해서 교차로 내부에 존재하는 쓰레드들의 threadlist에서 검색해 index값을 순서대로 저장한다.
	//threadlist[deadlock_recover_thread_list[i]]->tid 이런 식으로 사용, 왜냐하면 쓰레드에 대한 정보는 threadlist에 밖에 존재하지 않기 때문에
	int deadlock_recover_thread_list[8] = {0,};
	//위의 index 배열의 indexing을 위한 변수이다. 각 교차로 lock holder인 쓰레드를 threadlist에서 검색하면서 찾았을 경우 하나씩 증가시키는 변수
	int deadlock_recover_index = 0;

	//차량 쓰레드 생성후 vehicle_loop의 block되지 않을 쓰레드가 존재할 동안 blocked_thread_cnt == thread_cnt를 비교하면, 불상사가 발생할 수 있음.
	//사실 blocked_thread_cnt++ ,thread_cnt++을 같이 하기때문에 의미없는 조건임. crossroads에 manager thread를 생성한다면 쉽게 해결됨(thread_cnt를 미리 지정가능하기 때문)
	//다른 해결책을 내지 못한 아쉬운 부분
	//모든 쓰레드가 블럭될때까지 기다림.
	timer_msleep(1000);
	//첫 step을 시작하는 while문
	while(1){
		//전역변수 접근
		sema_down(&sem_TC);
			if (blocked_thread_cnt == thread_cnt){
				//모든 쓰레드가 블럭되었을경우 해제한다.
				for(int i = 0;i < thread_cnt; i++){
					if (threadlist[i] == NULL)
						break;
					thread_unblock(threadlist[i]);
					//이번 스텝에 blocked__threadlist로 저장된 쓰레드들을 초기화한다.
					blocked_threadlist[i] = NULL;
				}
				//blocked_thread의 개수를 초기화한다.
				blocked_thread_cnt = 0;
				//접근 종료
				sema_up(&sem_TC);
				break;
			}
		//접근 종료
		sema_up(&sem_TC);
		//다른 쓰레드의 접근을 위해 일정시간 텀을두고 접근시도
		timer_msleep(10);
	}
	//모든차량이 finished 될때까지 반복하는 while문
	while(1){

		//일단 block상태로 진입한다.
		sema_down(&sem_TRI);
		//마지막 step 차량 쓰레드에 의해 release 되면 상태확인후 차량 쓰레드의 block을 해제 시작
		printf("step:%d//",crossroads_step);
		//존재하는 쓰레드가 없으면 모든 차량이 완료됨으로 쓰레드 종료
		if(thread_cnt==0){
			break;
		}

		//위에서 설명함 매스텝 0 으로 초기화
		deadlock_recover_index = 0;

		//교차로 상태를 확인해 데드락 상태 여부를 저장하는 변수, true로 초기화 되고 deadlock조건에 하나라도 안맞으면 false
		bool deadlock_flag = true;
		//세로
		for(int i = 2;i<5;i++){
			//가로
			for(int j = 2;j<5;j++){
				//가운데는 넘어감
				if(i==3 && j==3)
					continue;

				//맵 lock의 holder 쓰레드가 있는지 확인 (차량이 존재하는 지 여부)
				if(maplock[i][j].holder){
					int indx = 0;
					//차량 쓰레드가 있을경우 threadlist에서 맵의 차량 쓰레드를 검색
					for (int k = 0;k<thread_cnt;k++){
						if(threadlist[k]->tid == maplock[i][j].holder->tid){
							//검색한 쓰레드 index를 교차로 차량 쓰레드 index를 저장하는 배열에 차례대로 저장한다.
							deadlock_recover_thread_list[deadlock_recover_index++] = k;
							indx = k;
							break;
						}
					}
					//.<<
					//...
					//...인 경우
					if(i==2 && (j==3 || j ==4)){
						//dest 가 A B
						if(!(destlist[indx] == 0 || destlist[indx] == 1)){
							deadlock_flag = false;
							i = 5;j = 5;
						}
					//...
					//...
					//>>.의 경우
					}else if(i==4 && (j==2||j==3)){
						//dest 가 C D
						if(!(destlist[indx] == 2 || destlist[indx] == 3)){
							deadlock_flag = false;
							i = 5;j = 5;
						}
					//v..
					//v..
					//...의 경우
					}else if(j==2 &&(i==2||i==3)){
						//dest 가 B C
						if(!(destlist[indx] == 1 || destlist[indx] == 2)){
							deadlock_flag = false;
							i = 5;j = 5;
						}
					//...
					//..^
					//..^의 경우
					}else if(j==4 && (i==3||i==4)){
						//dest 가 A D
						if(!(destlist[indx] == 0 || destlist[indx] == 3)){
							deadlock_flag = false;
							i = 5;j = 5;
						}
					}
					//차량 방향이 하나라도 데드락 조건에 맞지않는다면 데드락 상황x, for문을 빠져나옴

				//차량이 하나라도 존재하지 않는다면 데드락 상황x ,for문을 빠져나옴
				}else{
					deadlock_flag = false;
					i = 5;j = 5;
				}
			}
		}

		//dead lock 발생 및 회복 과정
		if (deadlock_flag){
			sema_down(&sem_TC);
			//차량 쓰레드에 데드락 상황을 알리기 위한 변수
			is_deadlock_recover = true;
			//세로
			for(int i = 2;i<5;i++){
				//가로
				for(int j = 2;j<5;j++){
					if(i==3 && j == 3)
						continue;
					//교차로 내의 모든 lock을 해제한다. (lock_release사용x)
					struct lock *_lock = &maplock[i][j];
					_lock->holder = NULL;
  					sema_up (&_lock->semaphore);
				}
			}
			//위에서 저장한 교차로에 존재하는 차량 쓰레드의 인덱스 배열을 통해 교차로 차량 쓰레드만 unblock
			for(int i = 0;i < 8;i++){
				thread_unblock(threadlist[deadlock_recover_thread_list[i]]);
			}
			//새로운 스텝을 위해 blocked_threadlist 초기화
			for(int i = 0;i < thread_cnt; i++){
				blocked_threadlist[i] = NULL;
			}
			//새로운 스텝을 위해 초기화
			blocked_thread_cnt = 0;
			//한 스텝 증가
			crossroads_step++;
			//접근 해제
			sema_up(&sem_TC);

		//데드락 상황 x, 일반적인 스텝 종료상황
		}else{
			
			//전역변수 접근
			sema_down(&sem_TC);
			//데드락 상황 아님을 알림
			is_deadlock_recover = false;
			//저장된 모든쓰레드의 block을 해제함, step이 끝나기 위해서는 모든 차량쓰레드가 blocked_threadlist에 추가되야 되기떄문에
			for(int i = 0;i < thread_cnt; i++){
				if (threadlist[i] == NULL)
					break;
				thread_unblock(threadlist[i]);
				//초기화작업
				blocked_threadlist[i] = NULL;
			}
			//초기화작업
			blocked_thread_cnt = 0;
			//한스텝 증가
			crossroads_step++;
			//접근 해제
			sema_up(&sem_TC);

		}

	}
	//쓰레드 종료
	return;
}
