use super::epoch_parser;

use bevy_egui::egui;
use ftime::{Duration, Epoch};
use std::ops::{Bound, Range, RangeBounds};

const POINTS_PER_STEP: f32 = 6.0;

pub trait Fields: Copy + Ord + Send + Sync + 'static {
    type Field: Copy + PartialEq + Send + Sync + 'static;

    fn text(self) -> String;

    fn fields(text: &str) -> Vec<(Self::Field, Range<usize>)>;

    fn step(self, field: Self::Field, steps: i64) -> Self;

    fn parse(text: &str) -> Option<Self>;
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CalendarField {
    Year,
    Month,
    Day,
    Hour,
    Minute,
    Second,
    Millisecond,
}

impl Fields for Epoch {
    type Field = CalendarField;

    #[inline]
    fn text(self) -> String {
        self.to_string()
    }

    fn fields(text: &str) -> Vec<(CalendarField, Range<usize>)> {
        let date = text.len().saturating_sub(13);
        let year = date.saturating_sub(6);
        vec![
            (CalendarField::Year, 0..year),
            (CalendarField::Month, year + 1..year + 3),
            (CalendarField::Day, year + 4..year + 6),
            (CalendarField::Hour, date + 1..date + 3),
            (CalendarField::Minute, date + 4..date + 6),
            (CalendarField::Second, date + 7..date + 9),
            (CalendarField::Millisecond, date + 10..date + 13),
        ]
    }

    fn step(self, field: CalendarField, steps: i64) -> Self {
        if steps == 0 {
            return self;
        }
        let seconds = |unit: f64| self + Duration::from_seconds(steps as f64 * unit);
        match field {
            CalendarField::Year => self.add_months(steps.saturating_mul(12)),
            CalendarField::Month => self.add_months(steps),
            CalendarField::Day => seconds(ftime::SEC_PER_DAY),
            CalendarField::Hour => seconds(ftime::SEC_PER_HOUR),
            CalendarField::Minute => seconds(ftime::SEC_PER_MIN),
            CalendarField::Second => seconds(1.0),
            CalendarField::Millisecond => seconds(ftime::SEC_PER_MILLI),
        }
    }

    #[inline]
    fn parse(text: &str) -> Option<Self> {
        epoch_parser(text)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DurationField {
    Day,
    Hour,
    Minute,
    Second,
    Millisecond,
}

impl DurationField {
    #[inline]
    fn seconds(self) -> f64 {
        match self {
            DurationField::Day => ftime::SEC_PER_DAY,
            DurationField::Hour => ftime::SEC_PER_HOUR,
            DurationField::Minute => ftime::SEC_PER_MIN,
            DurationField::Second => 1.0,
            DurationField::Millisecond => ftime::SEC_PER_MILLI,
        }
    }
}

impl Fields for Duration {
    type Field = DurationField;

    #[inline]
    fn text(self) -> String {
        self.expanded().to_string()
    }

    fn fields(text: &str) -> Vec<(DurationField, Range<usize>)> {
        let mut fields = vec![];
        let mut start = 0;
        for part in text.split(' ') {
            let digits = start + usize::from(part.starts_with('-'))..start + part.len();
            let digits = digits.start..digits.end.saturating_sub(1).max(digits.start);
            let number = &text[digits.clone()];
            if !number.is_empty() && number.bytes().all(|b| b.is_ascii_digit() || b == b'.') {
                match part.as_bytes().last() {
                    Some(b'd') => fields.push((DurationField::Day, digits)),
                    Some(b'h') => fields.push((DurationField::Hour, digits)),
                    Some(b'm') => fields.push((DurationField::Minute, digits)),
                    Some(b's') => {
                        if let Some(dot) = number.find('.').map(|dot| digits.start + dot) {
                            fields.push((DurationField::Second, digits.start..dot));
                            fields.push((DurationField::Millisecond, dot + 1..digits.end));
                        }
                    }
                    _ => (),
                }
            }
            start += part.len() + 1;
        }
        fields
    }

    #[inline]
    fn step(self, field: DurationField, steps: i64) -> Self {
        Duration::from_seconds(self.as_seconds() + steps as f64 * field.seconds())
    }

    #[inline]
    fn parse(text: &str) -> Option<Self> {
        text.parse().ok()
    }
}

#[derive(Clone, Copy)]
struct Drag<T: Fields> {
    field: T::Field,
    origin: T,
    points: f32,
}

#[must_use = "You should put this widget in a ui with `ui.add(widget);`"]
pub struct FieldDragValue<'a, T: Fields> {
    value: &'a mut T,
    range: (Bound<T>, Bound<T>),
}

pub type CalendarDragValue<'a> = FieldDragValue<'a, Epoch>;

pub type DurationDragValue<'a> = FieldDragValue<'a, Duration>;

impl<'a, T: Fields> FieldDragValue<'a, T> {
    pub fn new(value: &'a mut T) -> Self {
        Self {
            value,
            range: (Bound::Unbounded, Bound::Unbounded),
        }
    }

    pub fn range(mut self, range: impl RangeBounds<T>) -> Self {
        self.range = (range.start_bound().cloned(), range.end_bound().cloned());
        self
    }
}

impl<T: Fields> egui::Widget for FieldDragValue<'_, T> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let Self { value, range } = self;
        let clamp = |value: T| {
            let value = match range.0 {
                Bound::Included(min) | Bound::Excluded(min) => value.max(min),
                Bound::Unbounded => value,
            };
            match range.1 {
                Bound::Included(max) | Bound::Excluded(max) => value.min(max),
                Bound::Unbounded => value,
            }
        };
        let initial = *value;
        *value = clamp(*value);

        let id = ui.next_auto_id();
        let editing = ui.is_enabled()
            && ui.memory_mut(|memory| {
                memory.interested_in_focus(id, ui.layer_id());
                memory.has_focus(id)
            });

        let mut response = if editing {
            edit(ui, id, value, clamp)
        } else {
            drag(ui, id, value, clamp)
        };

        if *value != initial {
            response.mark_changed();
        }
        response.widget_info(|| {
            egui::WidgetInfo::labeled(egui::WidgetType::DragValue, ui.is_enabled(), value.text())
        });
        response
    }
}

pub fn epoch_placeholder(ui: &mut egui::Ui, text: &str) -> egui::Response {
    let text = egui::RichText::new(text).text_style(ui.style().drag_value_text_style.clone());
    let button = egui::Button::new(text).min_size(ui.spacing().interact_size);
    ui.add_enabled(false, button)
}

fn edit<T: Fields>(
    ui: &mut egui::Ui,
    id: egui::Id,
    value: &mut T,
    clamp: impl Fn(T) -> T,
) -> egui::Response {
    let shown = value.text();
    let mut text = ui
        .data_mut(|data| data.remove_temp::<String>(id))
        .unwrap_or_else(|| shown.clone());
    let padding = ui.spacing().button_padding;
    let response = ui.add(
        egui::TextEdit::singleline(&mut text)
            .id(id)
            .clip_text(false)
            .horizontal_align(ui.layout().horizontal_align())
            .vertical_align(ui.layout().vertical_align())
            .margin(padding)
            .min_size(ui.spacing().interact_size)
            .desired_width(ui.spacing().interact_size.x - 2.0 * padding.x)
            .font(ui.style().drag_value_text_style.clone()),
    );
    if response.gained_focus() {
        select_all(ui, id, &text);
    }

    if response.lost_focus() {
        if !ui.input(|i| i.key_pressed(egui::Key::Escape))
            && text != shown
            && let Some(typed) = T::parse(&text)
        {
            *value = clamp(typed);
        }
    } else {
        ui.data_mut(|data| data.insert_temp(id, text));
    }
    response
}

fn drag<T: Fields>(
    ui: &mut egui::Ui,
    id: egui::Id,
    value: &mut T,
    clamp: impl Fn(T) -> T,
) -> egui::Response {
    let shown = *value;
    let text = shown.text();
    let font = ui.style().drag_value_text_style.resolve(ui.style());
    let galley = ui
        .painter()
        .layout_no_wrap(text.clone(), font, egui::Color32::PLACEHOLDER);
    let padding = ui.spacing().button_padding;
    let size = (galley.size() + 2.0 * padding).max(ui.spacing().interact_size);
    let (rect, mut response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
    debug_assert_eq!(response.id, id);

    let text_pos = egui::Align2::CENTER_CENTER
        .anchor_size(rect.center(), galley.size())
        .min;
    let x = |index: usize| {
        text_pos.x
            + galley
                .pos_from_cursor(egui::text::CCursor::new(index))
                .min
                .x
    };
    let fields = T::fields(&text)
        .into_iter()
        .map(|(field, range)| (field, x(range.start)..=x(range.end)))
        .collect::<Vec<_>>();
    let field_at = |pos: egui::Pos2| {
        let distance = |range: &std::ops::RangeInclusive<f32>| {
            (range.start() - pos.x).max(pos.x - range.end()).max(0.0)
        };
        fields
            .iter()
            .min_by(|(_, a), (_, b)| distance(a).total_cmp(&distance(b)))
            .map(|&(field, _)| field)
    };

    if response.drag_started()
        && let Some(pointer) = ui.input(|i| i.pointer.press_origin())
        && let Some(field) = field_at(pointer)
    {
        let drag = Drag {
            field,
            origin: shown,
            points: 0.0,
        };
        ui.data_mut(|data| data.insert_temp(id, drag));
    }
    let drag = response
        .dragged()
        .then(|| ui.data(|data| data.get_temp::<Drag<T>>(id)))
        .flatten();
    if let Some(mut drag) = drag {
        let delta = response.drag_delta();
        drag.points += delta.x - delta.y;
        let steps = (drag.points / POINTS_PER_STEP).trunc() as i64;
        *value = clamp(drag.origin.step(drag.field, steps));
        ui.data_mut(|data| data.insert_temp(id, drag));
    }
    if response.drag_stopped() {
        ui.data_mut(|data| data.remove::<Drag<T>>(id));
    }

    let hovered = response.hover_pos().filter(|_| ui.is_enabled());
    let field = drag.map(|drag| drag.field).or(hovered.and_then(field_at));

    if response.clicked() {
        ui.data_mut(|data| data.remove::<String>(id));
        ui.memory_mut(|memory| memory.request_focus(id));
        select_all(ui, id, &text);
    }
    if response.hovered() || response.dragged() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
    }
    if ui.style().explanation_tooltips {
        response = response.on_hover_text("Drag a field to change it.\nClick to enter a value.");
    }

    if ui.is_rect_visible(rect) {
        let visuals = ui.style().interact(&response);
        let painter = ui.painter();
        painter.rect(
            rect.expand(visuals.expansion),
            visuals.corner_radius,
            visuals.weak_bg_fill,
            visuals.bg_stroke,
            egui::StrokeKind::Inside,
        );
        if let Some((_, x_range)) = fields.iter().find(|&&(f, _)| Some(f) == field) {
            let y_range = text_pos.y..=text_pos.y + galley.size().y;
            painter.rect_filled(
                egui::Rect::from_x_y_ranges(x_range.clone(), y_range).expand(1.0),
                visuals.corner_radius,
                ui.visuals().selection.bg_fill,
            );
        }
        painter.galley(text_pos, galley, visuals.text_color());
    }

    if *value != shown {
        ui.ctx().request_repaint();
    }
    response
}

fn select_all(ui: &egui::Ui, id: egui::Id, text: &str) {
    let mut state = egui::TextEdit::load_state(ui.ctx(), id).unwrap_or_default();
    state
        .cursor
        .set_char_range(Some(egui::text::CCursorRange::two(
            egui::text::CCursor::default(),
            egui::text::CCursor::new(text.chars().count()),
        )));
    state.store(ui.ctx(), id);
}
