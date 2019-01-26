# git の管理から除外するために，
# tmp 以下に全てをインストールする
WORK_PATH = ./tmp
RISCV_COMPLIANCE_PATH = $(WORK_PATH)/riscv-compliance

ENV_CFG = env.cfg

# ENV_CFG が存在してたら読み込む
ifneq ("$(wildcard $(ENV_CFG))","")
	include $(ENV_CFG)
endif


.DEFAULT_GOAL = all

#
# --- Clone riscv-compliance
#
$(ENV_CFG):
	@echo CC= > $(ENV_CFG)
	@echo Edit '$(ENV_CFG)' to set a cross compiler path to CC
	false

# checkout riscv-compliane
$(RISCV_COMPLIANCE_PATH): 
	mkdir $(WORK_PATH) -p
	cd $(WORK_PATH) ;\
		git clone https://github.com/riscv/riscv-compliance

distclean:
	rm $(WORK_PATH) -r -f


#
# --- Build binaries
#


BIN_FILES    = $(SRC_APPS:%=$(BIN_DIR)/%)
RESULT_FILES = $(SRC_APPS:%=$(RESULT_DIR)/%)
TEST_GOALS   = $(SRC_APPS:%=test-%)

# リファレンス出力

# ビルド設定
TARGET_DIR = ./target-onikiri2
XCFLAGS = -g -static -I $(TARGET_DIR) -I $(RISCV_COMPLIANCE_PATH)/riscv-test-env


all: init
	$(MAKE) -f $(firstword $(MAKEFILE_LIST)) test

init: $(ENV_CFG) $(RISCV_COMPLIANCE_PATH)

# ビルド
build: $(BIN_FILES) 

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# "|" が入っているのは，ディレクトリを order-only-prerequisites とするため
# そうしないと，中身のバイナリが更新されるたびに BIN_DIR 自体がが更新されたと
# みなされる
$(BIN_DIR)/%: $(SRC_DIR)/%.S $(TARGET_DIR)/* Makefile | $(BIN_DIR)
	$(CC) $(XCFLAGS) -o $@ $< $(TARGET_DIR)/main.c


# テスト
test: build
	rm $(RESULT_DIR) -r -f
	$(MAKE) -f $(firstword $(MAKEFILE_LIST)) $(RESULT_FILES)
	@echo "==== Test Successful (test-riscv-compliance) ===="

$(RESULT_DIR):
	mkdir -p $(RESULT_DIR)

$(RESULT_DIR)/%: | $(RESULT_DIR)
	../../project/gcc/onikiri2/a.out param.xml \
		-x /Session/Emulator/Processes/Process/@Command=$(BIN_DIR)/$(notdir $@) \
		-x /Session/Emulator/Processes/Process/@STDOUT=$@ \
		> $@.xml
	diff $(REF_DIR)/$(notdir $@).reference_output $@; 
	@echo Check $(notdir $@) ... OK

.PHONY: $(TEST_GOALS)
$(TEST_GOALS): 
	$(MAKE) -f $(firstword $(MAKEFILE_LIST)) $(BIN_DIR)/$(patsubst test-%,%,$@)
	rm $(RESULT_DIR)/$(patsubst test-%,%,$@) -f
	$(MAKE) -f $(firstword $(MAKEFILE_LIST)) $(RESULT_DIR)/$(patsubst test-%,%,$@)

clean:
	rm $(BIN_DIR) -f -r
	rm $(RESULT_DIR) -r -f
