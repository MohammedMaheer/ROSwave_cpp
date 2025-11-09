#include "../include/gui/advanced_filter.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QInputDialog>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>

AdvancedFilterWidget::AdvancedFilterWidget(QWidget* parent)
    : QWidget(parent), cross_tab_search_enabled_(false) {
    setup_ui();
}

void AdvancedFilterWidget::setup_ui() {
    auto main_layout = new QVBoxLayout(this);

    // ========== Search Input ==========
    auto search_layout = new QHBoxLayout;
    auto search_label = new QLabel("🔍 Search:");
    search_input_ = new QLineEdit;
    search_input_->setPlaceholderText(
        "Search topics (tip: use 'r:pattern' for regex)");
    connect(search_input_, &QLineEdit::textChanged, this,
            &AdvancedFilterWidget::on_search_text_changed);
    search_layout->addWidget(search_label);
    search_layout->addWidget(search_input_, 1);
    main_layout->addLayout(search_layout);

    // ========== Filter Options ==========
    auto filter_layout = new QHBoxLayout;

    // Filter field
    auto field_label = new QLabel("Filter by:");
    filter_field_combo_ = new QComboBox;
    filter_field_combo_->addItems(
        {"All Fields", "Name", "Type", "Rate", "Status"});
    connect(filter_field_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &AdvancedFilterWidget::on_filter_field_changed);
    filter_layout->addWidget(field_label);
    filter_layout->addWidget(filter_field_combo_);

    // Regex mode
    regex_checkbox_ = new QCheckBox("Regular Expression");
    connect(regex_checkbox_, &QCheckBox::toggled, this,
            &AdvancedFilterWidget::on_regex_toggled);
    filter_layout->addWidget(regex_checkbox_);

    filter_layout->addSpacing(20);

    // Rate range
    auto rate_label = new QLabel("Rate (Hz):");
    min_rate_spin_ = new QDoubleSpinBox;
    min_rate_spin_->setMinimum(0);
    min_rate_spin_->setMaximum(1e6);
    min_rate_spin_->setValue(0);
    min_rate_spin_->setPrefix("≥ ");
    connect(min_rate_spin_,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &AdvancedFilterWidget::on_min_rate_changed);

    auto dash_label = new QLabel(" - ");

    max_rate_spin_ = new QDoubleSpinBox;
    max_rate_spin_->setMinimum(0);
    max_rate_spin_->setMaximum(1e6);
    max_rate_spin_->setValue(1e6);
    max_rate_spin_->setPrefix("≤ ");
    connect(max_rate_spin_,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &AdvancedFilterWidget::on_max_rate_changed);

    filter_layout->addWidget(rate_label);
    filter_layout->addWidget(min_rate_spin_);
    filter_layout->addWidget(dash_label);
    filter_layout->addWidget(max_rate_spin_);

    filter_layout->addSpacing(20);

    // Status filter
    auto status_label = new QLabel("Status:");
    status_combo_ = new QComboBox;
    status_combo_->addItems({"All", "Active", "Inactive", "Error"});
    connect(status_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &AdvancedFilterWidget::on_status_changed);
    filter_layout->addWidget(status_label);
    filter_layout->addWidget(status_combo_);

    // Clear button
    clear_btn_ = new QPushButton("🗑️ Clear");
    connect(clear_btn_, &QPushButton::clicked, this,
            &AdvancedFilterWidget::on_clear_clicked);
    filter_layout->addWidget(clear_btn_);

    filter_layout->addStretch();
    main_layout->addLayout(filter_layout);

    // ========== Filter Presets ==========
    auto preset_layout = new QHBoxLayout;
    auto preset_label = new QLabel("💾 Presets:");
    preset_combo_ = new QComboBox;
    preset_combo_->addItem("New...");
    connect(preset_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &AdvancedFilterWidget::on_preset_selected);
    preset_layout->addWidget(preset_label);
    preset_layout->addWidget(preset_combo_);

    save_preset_btn_ = new QPushButton("Save");
    connect(save_preset_btn_, &QPushButton::clicked, this,
            &AdvancedFilterWidget::on_save_preset_clicked);
    preset_layout->addWidget(save_preset_btn_);

    delete_preset_btn_ = new QPushButton("Delete");
    connect(delete_preset_btn_, &QPushButton::clicked, this,
            &AdvancedFilterWidget::on_delete_preset_clicked);
    preset_layout->addWidget(delete_preset_btn_);

    preset_layout->addStretch();
    main_layout->addLayout(preset_layout);

    setLayout(main_layout);
}

AdvancedFilterWidget::FilterCriteria
AdvancedFilterWidget::get_filter_criteria() const {
    return current_criteria_;
}

void AdvancedFilterWidget::set_filter_fields(const QStringList& fields) {
    filter_field_combo_->clear();
    filter_field_combo_->addItem("All Fields");
    filter_field_combo_->addItems(fields);
}

void AdvancedFilterWidget::add_to_search_history(const QString& query) {
    if (query.isEmpty()) return;

    search_history_.removeAll(query);
    search_history_.prepend(query);

    if (search_history_.size() > MAX_HISTORY_SIZE) {
        search_history_.removeLast();
    }
}

QStringList AdvancedFilterWidget::get_search_history() const {
    return search_history_;
}

void AdvancedFilterWidget::clear_search_history() {
    search_history_.clear();
}

void AdvancedFilterWidget::save_filter_preset(const QString& preset_name,
                                             const FilterCriteria& criteria) {
    filter_presets_[preset_name] = criteria;
    preset_combo_->addItem(preset_name);
}

bool AdvancedFilterWidget::load_filter_preset(const QString& preset_name) {
    auto it = filter_presets_.find(preset_name);
    if (it == filter_presets_.end()) {
        return false;
    }

    current_criteria_ = it.value();

    // Update UI
    search_input_->setText(current_criteria_.search_text);
    filter_field_combo_->setCurrentText(current_criteria_.filter_field);
    regex_checkbox_->setChecked(current_criteria_.use_regex);
    min_rate_spin_->setValue(current_criteria_.min_rate);
    max_rate_spin_->setValue(current_criteria_.max_rate);
    status_combo_->setCurrentText(current_criteria_.status_filter);

    emit filter_changed(current_criteria_);
    emit preset_selected(preset_name);
    return true;
}

QStringList AdvancedFilterWidget::get_filter_presets() const {
    return filter_presets_.keys();
}

void AdvancedFilterWidget::delete_filter_preset(const QString& preset_name) {
    filter_presets_.remove(preset_name);
    int idx = preset_combo_->findText(preset_name);
    if (idx >= 0) {
        preset_combo_->removeItem(idx);
    }
}

void AdvancedFilterWidget::clear_filters() {
    search_input_->clear();
    filter_field_combo_->setCurrentIndex(0);
    regex_checkbox_->setChecked(false);
    min_rate_spin_->setValue(0);
    max_rate_spin_->setValue(1e6);
    status_combo_->setCurrentIndex(0);
}

void AdvancedFilterWidget::set_cross_tab_search_enabled(bool enabled) {
    cross_tab_search_enabled_ = enabled;
}

bool AdvancedFilterWidget::matches_criteria(const QString& text,
                                           const FilterCriteria& criteria) const {
    // Empty text matches everything
    if (criteria.search_text.isEmpty()) {
        return true;
    }

    // Regex matching
    if (criteria.use_regex) {
        QRegularExpression regex(criteria.search_text);
        return regex.match(text).hasMatch();
    }

    // String contains matching
    return text.contains(criteria.search_text, Qt::CaseInsensitive);
}

void AdvancedFilterWidget::on_search_text_changed(const QString& text) {
    // Parse search text for regex prefix
    if (text.startsWith("r:", Qt::CaseInsensitive)) {
        QString pattern = text.mid(2);
        if (validate_regex_pattern(pattern)) {
            regex_checkbox_->setChecked(true);
            current_criteria_.search_text = pattern;
        }
    } else {
        regex_checkbox_->setChecked(false);
        current_criteria_.search_text = text;
    }

    update_criteria();
}

void AdvancedFilterWidget::on_filter_field_changed(int index) {
    Q_UNUSED(index);
    current_criteria_.filter_field = filter_field_combo_->currentText();
    update_criteria();
}

void AdvancedFilterWidget::on_regex_toggled(bool checked) {
    current_criteria_.use_regex = checked;
    update_criteria();
}

void AdvancedFilterWidget::on_min_rate_changed(double value) {
    current_criteria_.min_rate = value;
    update_criteria();
}

void AdvancedFilterWidget::on_max_rate_changed(double value) {
    current_criteria_.max_rate = value;
    update_criteria();
}

void AdvancedFilterWidget::on_status_changed(int index) {
    Q_UNUSED(index);
    current_criteria_.status_filter = status_combo_->currentText();
    update_criteria();
}

void AdvancedFilterWidget::on_clear_clicked() {
    clear_filters();
    emit filter_changed(current_criteria_);
}

void AdvancedFilterWidget::on_preset_selected(int index) {
    Q_UNUSED(index);
    if (index <= 0) return;  // Skip "New..." option

    QString preset_name = preset_combo_->currentText();
    load_filter_preset(preset_name);
}

void AdvancedFilterWidget::on_save_preset_clicked() {
    bool ok;
    QString preset_name = QInputDialog::getText(
        this, "Save Filter Preset", "Enter preset name:", QLineEdit::Normal,
        "", &ok);

    if (ok && !preset_name.isEmpty()) {
        save_filter_preset(preset_name, current_criteria_);
        QMessageBox::information(this, "Success",
                               QString("Preset '%1' saved!").arg(preset_name));
    }
}

void AdvancedFilterWidget::on_delete_preset_clicked() {
    QString preset_name = preset_combo_->currentText();
    if (preset_name.isEmpty() || preset_name == "New...") {
        QMessageBox::warning(this, "Error", "Please select a preset to delete.");
        return;
    }

    if (QMessageBox::question(this, "Confirm Delete",
                             QString("Delete preset '%1'?").arg(preset_name)) ==
        QMessageBox::Yes) {
        delete_filter_preset(preset_name);
    }
}

void AdvancedFilterWidget::update_criteria() {
    emit filter_changed(current_criteria_);
}

bool AdvancedFilterWidget::validate_regex_pattern(const QString& pattern) const {
    QRegularExpression regex(pattern);
    return regex.isValid();
}

